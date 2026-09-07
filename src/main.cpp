#include <Arduino.h>
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>

/* ================= IO32 模拟振动通道(ADC)需求规格 =====================
 * 本节是 IO32 模拟振动检测功能的规格存档;以后这部分需求有变动,
 * 必须同步改这段注释,不要只改代码。
 *
 * 1) 串口发 's'/'S' 启动 ADC 采样,满幅 0V~3.3V。
 * 2) 空闲(无振动)= 高电平 → 定义为 0%;持续大振动 = 接近 0V → 定义为 100%。
 * 3) 0% 基线标定:'s' 之后先连续采样 ADC_CAL_MS(10 s),统计均值 mean 和
 *    峰峰值噪声半幅 noiseAmp(仅打印展示,不进入下面的计算)。标定一结束就打印:
 *      ADC calibrated: idle=...V noise=+-...V (n=...)
 *      1) -0.010V set to zero(0%)
 *      2) -0.020V set to zero(0%)
 *      ...(依次 +10mV 一档)
 *      10) -0.100V set to zero(0%)
 *      send 1~10 + enter to select
 *    进入 AWAITING_LEVEL 状态等用户选;串口发 '1'~'10' 选定挡位 N 后,
 *      zero(0%) = mean - N * ADC_ZERO_LEVEL_STEP_V(默认每档 10mV,范围 -10mV~-100mV)
 *    电压 > zero(0%) 都记为 0%,然后才真正转入 RUNNING 开始输出事件行。
 *    另设一个绝对期望值 ADC_NOMINAL_IDLE_V;若标定出的 mean 偏离它超过
 *    ADC_DRIFT_WARN_V(例:期望 3.13V,标定出 3.07V),判定为"可用但异常",
 *    在挡位菜单之前先发一条英文 WARNING 提示(单独另起一行:第一行数值,第二行
 *    "- still usable but check sensor/wiring")。
 * 4) 100% 基线:ADC 最低点不一定是 0V,固定 < ADC_LOW_FLOOR_V(100mV)记为
 *    100%;threshold(0%) 与 ADC_LOW_FLOOR_V 之间做线性映射得到 0~100%。
 * 5) 输出行格式 "ccc tt.ttttS a.aaV vvvv xx%"(例:04 23.2311S 2.21V 4012 23%):
 *      ccc      = 事件计数器,2 位起,从 1 开始,'s'/'r' 时复位
 *      tt.ttttS = 距最近一条 "time:" 整分基准的秒偏移,4 位小数(0.1ms 分辨率)+ 'S'
 *      a.aaV    = 校准后的电压值(analogReadMilliVolts,2 位小数),背景参考值
 *      vvvv     = 该次 ADC 原始采样值(analogRead 原始 0~4095 计数),背景参考值
 *      xx%      = 该次采样幅度百分比(0~100,四舍五入取整,由 a.aaV 换算而来)
 *    只有幅度 > 1% 的采样才输出一行,采样率越高、脉冲越宽,输出行数越多
 *    (例:采样周期 50µs、脉冲 >1% 宽度 400µs → 理论应输出 8 行;但实机二分法测出
 *    esp_timer 硬上限跟每次回调干的活直接相关——只读校准电压时上限在 120~135µs,
 *    多读一次 raw 原始值后上限退到 220~250µs,超过就必炸 task_wdt 重启,50µs 不可行,
 *    默认改用留有余量的 400µs/2.5kHz,见 ADC_SAMPLE_US 的注释)。空闲(≤1%)不输出,
 *    不占用串口带宽。
 * 6) 'T' 设置时钟、每整分播报一次 "time: ..." 的逻辑不变;只在这行末尾追加
 *    当前(最新一次采样)的电压/原始值/幅度,例如
 *    "time: 2026-09-07 14:23:00 3.19V 3958 0%"。
 * 7) 心跳 LED(LED_PIN)保持 1s 翻转不变;检测到振动(>1%)时同一颗 LED 会被强制
 *    点亮(不打断心跳节奏,只是暂时盖过它的输出),要求尽量实时,所以不经过 loop()
 *    轮询——直接在 adcTimerCallback 里 digitalWrite(HIGH),并用一个独立的一次性
 *    esp_timer(ledOffTimer)在 pct*LED_EVENT_US_PER_PCT 微秒后精确关灯(默认每 1%
 *    对应 10µs,100% → 1ms);期间只要还有新样本 >1%,就重新定时(相当于续时,
 *    不会中途被更短的新样本提前关掉)。
 *
 * IO33 与 IO32 共用同一个传感器插头,现场已把 IO33 接到 GND,固件把它设为
 * 纯输入(不驱动、不加内部上拉),避免和外部 GND 对冲。
 *
 * 原来 IO13 上的 SQ-SEN-200 数字振动传感器通道已完全移除(不再需要),
 * 现在只有 IO32 这一路模拟通道。
 * ===================================================================== */

#define FW_VERSION "Piezo VBR-Sen ver1.8.1"   // 固件版本(每次改动由 Claude 递增)

// ---------------- 配置 ----------------
constexpr int      LED_PIN         = 23;    // 心跳 LED,1 s 翻转一次,用来判断 MCU 是否活着;
                                             // 检测到振动(>1%)时同一颗 LED 会被强制点亮一段时间,
                                             // 时长正比于幅度(见 LED_EVENT_US_PER_PCT),不影响心跳节奏本身
constexpr uint32_t LED_INTERVAL_MS = 1000;
constexpr uint32_t LED_EVENT_US_PER_PCT = 10;  // 振动指示:每 1% 幅度对应常亮多少 µs(100% → 1000µs = 1ms)
                                                // 注:1ms 已经短到肉眼基本看不出"点亮"了,更适合示波器/逻辑分析仪测量
constexpr uint32_t TICK_US         = 100;   // 时间计数器分辨率 = 0.1 ms(时间戳单位 = 100µs)
constexpr uint32_t TX_BUF_BYTES    = 2048;  // 串口发送软缓冲
constexpr uint32_t CAL_MONITOR_MS  = 200;   // 'cal' 校准调试模式下打印 raw/电压 的间隔,避免刷屏

// ---------------- ADC 模拟振动通道配置(IO32) ----------------
constexpr int      ADC_PIN            = 32;      // 模拟振动传感器输入(IO32, ADC1_CH4)
constexpr int      ADC_GND_GUARD_PIN  = 33;      // 现场接了 GND 的邻脚(同一插头),只设输入,不驱动/不上拉
constexpr uint32_t ADC_SAMPLE_US      = 400;     // 采样周期 400µs = 2.5kHz。esp_timer 回调跑在专用任务里,
                                                  // 每次回调读 2 次 ADC(raw + 校准电压,见下方回调函数),
                                                  // 实机二分法测过硬上限:只读 1 次电压时上限在 120~135µs,
                                                  // 加上 raw 这次读取后开销翻倍,上限退到 220~250µs 之间,
                                                  // 超过就必炸 task_wdt(CPU 被占满,IDLE0 喂不上看门狗)。
                                                  // 400µs 离那条线还留 ~1.6 倍余量,300/400µs 均实测跑满
                                                  // 10s 标定无异常;以后再加别的每采样开销,务必重新实测
constexpr uint32_t ADC_CAL_MS         = 10000;   // 's' 后先花 10s 标定"空闲高电平=0%"基线

// 0% 基线 = 标定均值 - 挡位选中的偏移量。10s 标定结束后打印 1~10 挡菜单(每挡都是
// 均值再减一个整数倍的 ADC_ZERO_LEVEL_STEP_V,即 10mV~100mV),串口发 '1'~'10' + 回车选定,当场生效。
constexpr float    ADC_ZERO_LEVEL_STEP_V = 0.01f;   // 每一挡的偏移量,挡位 N(1~10)→ 偏移 = N * 该值
constexpr int      ADC_ZERO_LEVEL_COUNT  = 10;      // 挡位总数
constexpr float    ADC_LOW_FLOOR_V    = 0.100f;  // 低于此电压(100mV)固定记为 100%
constexpr float    ADC_NOMINAL_IDLE_V = 3.130f;  // 期望的空闲电压(现场实测值),标定值偏离它超过下面阈值就报警
constexpr float    ADC_DRIFT_WARN_V   = 0.05f;   // 允许的漂移范围(V)
constexpr uint32_t ADC_BUF_SIZE       = 2048;    // ADC 事件发送环形缓冲(2 的幂),吸收一次振动事件的突发采样

static bool timeIsSet    = false;  // 是否已用 'T' 命令设过墙上时间
static long lastTimeMin  = -1;     // 上次播报的"墙上分钟号"(epoch/60),用于整分触发一次

static bool     calMode         = false;  // 'cal' 命令/开机自检触发的校准调试模式:loop() 里周期打印 raw/电压
static uint32_t calMonitorLastMs = 0;

static volatile int64_t anchorEspUs   = 0;      // 锚点:esp_timer 读数
static volatile int64_t anchorEpochUs = 0;      // 锚点:同一时刻的墙上时间(µs since epoch)

// 记录 esp_timer 与墙上时间的对应关系,ADC 定时器回调靠它推算每个采样的绝对时刻
void captureAnchor() {
  int64_t esp = esp_timer_get_time();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  anchorEspUs   = esp;
  anchorEpochUs = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

// ---------------- ADC 模拟振动通道(IO32,esp_timer 定时采样,单独一套环形缓冲) ----------------
enum class AdcPhase : uint8_t { IDLE, CALIBRATING, AWAITING_LEVEL, RUNNING };

// t = 距整分偏移(单位 TICK_US);raw = ADC 原始采样值(0~4095);centivolt = 校准后电压*100(2 位小数);pct = 幅度 0~100
struct AdcSample { uint32_t t; uint16_t raw; uint16_t centivolt; uint8_t pct; };
static AdcSample adcRing[ADC_BUF_SIZE];
static volatile uint32_t adcHead    = 0;         // adc 定时器回调写
static volatile uint32_t adcTail    = 0;         // loop 读
static volatile uint32_t adcDropped = 0;         // 缓冲满导致的丢失计数
static uint32_t adcSendIdx = 1;                  // 发送端计数(= 输出里的 ccc),只有 loop 碰

static volatile AdcPhase adcPhase        = AdcPhase::IDLE;
static volatile float    adcZeroV        = 0;    // 标定得到的 0% 电压
static volatile uint8_t  adcLastPct      = 0;    // 最新一次采样的幅度(供 "time:" 行末尾展示)
static volatile uint16_t adcLastRaw      = 0;    // 最新一次采样的原始 ADC 值(同上)
static volatile uint16_t adcLastCentivolt = 0;   // 最新一次采样的电压*100,2 位小数(供事件行/time: 行用)
static volatile uint16_t adcLastMillivolt = 0;   // 最新一次采样的电压*1000,3 位小数(供 'cal' 调试打印用)

// 振动指示灯:硬件定时器驱动,不经过 loop() 轮询,做到尽量实时。
// adcTimerCallback 检测到 >1% 就直接 digitalWrite(HIGH),同时用一个独立的一次性
// esp_timer(ledOffTimer)在 pct*LED_EVENT_US_PER_PCT 微秒后精确关灯;期间新样本
// 只要还 >1% 就重新定时(相当于"续命"),灯就一直亮到最后一次达标采样对应的时长结束。
static volatile bool heartbeatLedState = false;  // 心跳"此刻应该"是什么电平,loop() 按 1s 周期翻转
static volatile bool ledForcedOn      = false;   // true = 正被振动强制点亮,loop() 心跳翻转时不要碰引脚
static esp_timer_handle_t ledOffTimer = nullptr;

// ledOffTimer 到点回调:交还心跳控制权
void ledOffTimerCallback(void *) {
  ledForcedOn = false;
  digitalWrite(LED_PIN, heartbeatLedState);
}

static esp_timer_handle_t adcTimer = nullptr;
static int64_t  adcCalStartUs = 0;
static double   adcCalSum     = 0;
static float    adcCalMin     = 0;
static float    adcCalMax     = 0;
static uint32_t adcCalCount   = 0;
static float    adcCalMean    = 0;   // 标定均值,标定结束后保留,供 1~10 选档时计算 zero 用

// 10s 标定窗口结束:先停采样定时器,打印标定摘要 + 1~10 挡菜单,等用户选档(见 selectZeroLevel)
void adcFinishCalibration() {
  float mean = (adcCalCount > 0) ? (float)(adcCalSum / adcCalCount) : 0.0f;
  float noiseAmp = (adcCalMax - adcCalMin) / 2.0f;
  adcCalMean = mean;
  esp_timer_stop(adcTimer);            // 挡位还没选定,先不采样,省得空转
  adcPhase = AdcPhase::AWAITING_LEVEL;

  char line[96];
  snprintf(line, sizeof(line), "ADC calibrated: idle=%.3fV noise=+-%.3fV (n=%lu)",
           mean, noiseAmp, (unsigned long)adcCalCount);
  Serial.println(line);

  float drift = mean - ADC_NOMINAL_IDLE_V;
  float absDrift = drift < 0 ? -drift : drift;
  if (absDrift > ADC_DRIFT_WARN_V) {
    char warn[112];
    snprintf(warn, sizeof(warn),
             "WARNING: ADC idle level drifted to %.3fV (nominal %.3fV, delta %.3fV)",
             mean, ADC_NOMINAL_IDLE_V, drift);
    Serial.println(warn);
    Serial.println("- still usable but check sensor/wiring");
  }

  for (int level = 1; level <= ADC_ZERO_LEVEL_COUNT; level++) {
    char opt[48];
    snprintf(opt, sizeof(opt), "%d) -%.3fV set to zero(0%%)", level, level * ADC_ZERO_LEVEL_STEP_V);
    Serial.println(opt);
  }
  char prompt[32];
  snprintf(prompt, sizeof(prompt), "send 1~%d + enter to select", ADC_ZERO_LEVEL_COUNT);
  Serial.println(prompt);
}

// 收到 '1'~'10':选定挡位,zero(0%) = 标定均值 - level*ADC_ZERO_LEVEL_STEP_V,然后正式转入 RUNNING
void selectZeroLevel(int level) {
  if (adcPhase != AdcPhase::AWAITING_LEVEL) {
    Serial.println("not waiting for a zero-level selection right now");
    return;
  }
  if (level < 1 || level > ADC_ZERO_LEVEL_COUNT) {
    Serial.println("bad level, send 1~10");
    return;
  }
  float offset = level * ADC_ZERO_LEVEL_STEP_V;
  float zero = adcCalMean - offset;
  if (zero < ADC_LOW_FLOOR_V + 0.05f) zero = ADC_LOW_FLOOR_V + 0.05f;  // 兜底,避免分母太小/为负
  adcZeroV = zero;
  adcPhase = AdcPhase::RUNNING;
  esp_timer_start_periodic(adcTimer, ADC_SAMPLE_US);   // 挡位选完,重新开始采样

  char line[80];
  snprintf(line, sizeof(line), "zero(0%%) set to %.3fV (level %d, -%.3fV)", zero, level, offset);
  Serial.println(line);
}

// esp_timer 周期回调(任务上下文,非真正 ISR,可以放心调 analogRead/Serial):每 ADC_SAMPLE_US 跑一次
void adcTimerCallback(void *) {
  int      raw = analogRead(ADC_PIN);              // 原始采样值 0~4095,单独读一次(用于调试展示)
  float    v   = analogReadMilliVolts(ADC_PIN) / 1000.0f;  // 校准后电压,百分比计算仍然只认这个

  if (adcPhase == AdcPhase::CALIBRATING) {
    adcLastRaw       = (uint16_t)raw;              // 标定期间也更新电压/原始值,方便 "time:" 行实时展示
    adcLastCentivolt = (uint16_t)(v * 100.0f + 0.5f);  // 百分比还没有基线可算,保持 0 不动
    adcLastMillivolt = (uint16_t)(v * 1000.0f + 0.5f);
    adcCalSum += v;
    adcCalCount++;
    if (v < adcCalMin) adcCalMin = v;
    if (v > adcCalMax) adcCalMax = v;
    if (esp_timer_get_time() - adcCalStartUs >= (int64_t)ADC_CAL_MS * 1000LL) {
      adcFinishCalibration();
    }
    return;
  }

  if (adcPhase != AdcPhase::RUNNING) return;

  float denom = adcZeroV - ADC_LOW_FLOOR_V;
  if (denom < 0.05f) denom = 0.05f;
  float pctF = 100.0f * (adcZeroV - v) / denom;
  if (pctF < 0)   pctF = 0;
  if (pctF > 100) pctF = 100;
  uint8_t  pct        = (uint8_t)(pctF + 0.5f);
  uint16_t centivolt  = (uint16_t)(v * 100.0f + 0.5f);
  adcLastPct       = pct;
  adcLastRaw       = (uint16_t)raw;
  adcLastCentivolt = centivolt;
  adcLastMillivolt = (uint16_t)(v * 1000.0f + 0.5f);

  if (pct > 1) {                                          // 只有 >1% 才算一次事件,进缓冲等待发送
    int64_t now = esp_timer_get_time();

    if (!ledForcedOn) {                                    // 振动指示灯:硬件定时器驱动,实时点亮/续时
      ledForcedOn = true;
      digitalWrite(LED_PIN, HIGH);
    }
    esp_timer_stop(ledOffTimer);                            // 忽略返回值:没在跑也没关系
    esp_timer_start_once(ledOffTimer, (uint64_t)pct * LED_EVENT_US_PER_PCT);  // 幅度越大续时越久

    int64_t epochUs = anchorEpochUs + (now - anchorEspUs); // 用同一套时钟锚点换算绝对时刻
    uint32_t t = (uint32_t)((epochUs % 60000000LL) / TICK_US);

    uint32_t next = (adcHead + 1) & (ADC_BUF_SIZE - 1);
    if (next == adcTail) {
      adcDropped++;                                       // 缓冲满,丢弃
    } else {
      adcRing[adcHead].t         = t;
      adcRing[adcHead].raw       = (uint16_t)raw;
      adcRing[adcHead].centivolt = centivolt;
      adcRing[adcHead].pct       = pct;
      adcHead = next;
    }
  }
}

// 创建(首次)/ 重新启动周期采样定时器
void adcTimerStart() {
  if (adcTimer == nullptr) {
    esp_timer_create_args_t args = {};
    args.callback = &adcTimerCallback;
    args.dispatch_method = ESP_TIMER_TASK;   // 任务上下文回调,不是 ISR,可以调 Serial/analogRead
    args.name = "adc_sample";
    esp_timer_create(&args, &adcTimer);
  }
  esp_timer_start_periodic(adcTimer, ADC_SAMPLE_US);
}

// 收到 's'/'S':复位并重新标定 0% 基线,然后开始采样
void adcStartCalibration() {
  if (adcTimer) esp_timer_stop(adcTimer);   // 若之前在跑,先停下来再复位,避免和回调打架
  adcHead = adcTail = 0;
  adcDropped = 0;
  adcSendIdx = 1;
  adcCalSum = 0;
  adcCalCount = 0;
  adcCalMin = 1e9f;
  adcCalMax = -1e9f;
  adcLastPct = 0;
  adcPhase = AdcPhase::CALIBRATING;
  adcCalStartUs = esp_timer_get_time();
  adcTimerStart();
  Serial.println("ADC calibrating idle level for 10 s, keep sensor still...");
}

// 收到 'p'/'P':停止采样
void adcStop() {
  if (adcTimer) esp_timer_stop(adcTimer);
  adcPhase = AdcPhase::IDLE;
}

// 收到 'r'/'R':只清事件缓冲 + 计数(不重新标定);运行中就短暂停一下定时器避免和回调抢缓冲
void adcResetRuntime() {
  bool wasRunning = (adcPhase != AdcPhase::IDLE);
  if (wasRunning && adcTimer) esp_timer_stop(adcTimer);
  adcHead = adcTail = 0;
  adcDropped = 0;
  adcSendIdx = 1;
  if (wasRunning) esp_timer_start_periodic(adcTimer, ADC_SAMPLE_US);
}

// 非阻塞发送一行 ADC 事件,格式 "ccc tt.ttttS a.aaV vvvv xx%"
void sendAdcLine() {
  if (adcTail == adcHead) return;
  AdcSample s = adcRing[adcTail];
  char line[48];
  int n = snprintf(line, sizeof(line), "%02lu %lu.%04luS %u.%02uV %u %u%%\n",
                   (unsigned long)adcSendIdx,
                   (unsigned long)(s.t / 10000), (unsigned long)(s.t % 10000),
                   (unsigned)(s.centivolt / 100), (unsigned)(s.centivolt % 100),
                   (unsigned)s.raw, (unsigned)s.pct);
  if (Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t *)line, n);
    adcTail = (adcTail + 1) & (ADC_BUF_SIZE - 1);
    adcSendIdx++;
  }
}

// 把当前墙上时间格式化成 "YYYY-MM-DD HH:MM:SS"
void fmtNow(char *out, size_t n) {
  time_t now = time(NULL);
  struct tm tm;
  localtime_r(&now, &tm);
  strftime(out, n, "%Y-%m-%d %H:%M:%S", &tm);
}

// 把 epoch 秒向下取整到整分,格式化成 "YYYY-MM-DD HH:MM:00"(采样偏移的基准)
void fmtMinute(char *out, size_t n, time_t t) {
  time_t m = (t / 60) * 60;
  struct tm tm;
  localtime_r(&m, &tm);
  strftime(out, n, "%Y-%m-%d %H:%M:%S", &tm);
}

// 播报一行基准时间:"time: YYYY-MM-DD HH:MM:00 a.aaV vvvv xx%"(末尾是当前 ADC 电压/原始值/幅度),并记住这一分钟
void emitTimeBase() {
  time_t now = time(NULL);
  lastTimeMin = (long)(now / 60);
  char buf[24];
  fmtMinute(buf, sizeof(buf), now);
  uint16_t cv = adcLastCentivolt;
  char tail[24];
  snprintf(tail, sizeof(tail), " %u.%02uV %u %u%%",
           (unsigned)(cv / 100), (unsigned)(cv % 100),
           (unsigned)adcLastRaw, (unsigned)adcLastPct);
  Serial.print("time: ");
  Serial.print(buf);
  Serial.println(tail); 
}

// 收到 'T YYYYMMDD HHMMSS':设定墙上时间
void setTimeCmd(const char *s) {
  int Y, Mo, Da, H, Mi, Se;
  if (sscanf(s, "T %4d%2d%2d %2d%2d%2d", &Y, &Mo, &Da, &H, &Mi, &Se) != 6) {
    Serial.println("bad time, use: T YYYYMMDD HHMMSS");
    return;
  }
  struct tm tm = {0};
  tm.tm_year = Y - 1900;
  tm.tm_mon  = Mo - 1;
  tm.tm_mday = Da;
  tm.tm_hour = H;
  tm.tm_min  = Mi;
  tm.tm_sec  = Se;
  time_t epoch = mktime(&tm);
  if (epoch == (time_t)-1) { Serial.println("bad time value"); return; }

  struct timeval tv;
  tv.tv_sec  = epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);
  timeIsSet = true;
  lastTimeMin = (long)(epoch / 60);           // 当前这一分钟不补播,下一整分才播

  // 刷新 ADC 用的时间锚点;若正在标定/采集,短暂停一下定时器,避免和回调撞上导致 64 位撕裂读
  bool adcRunning = (adcPhase != AdcPhase::IDLE);
  if (adcRunning && adcTimer) esp_timer_stop(adcTimer);
  captureAnchor();
  if (adcRunning) esp_timer_start_periodic(adcTimer, ADC_SAMPLE_US);

  char buf[24];
  fmtNow(buf, sizeof(buf));
  Serial.print("time set: ");
  Serial.println(buf);
}

// 收到 'cal':纯校准调试模式,不需要先设时钟。做一次 10s 的 0% 标定,期间由 loop()
// 周期打印 raw/电压(见 CAL_MONITOR_MS);标定一结束(转入 RUNNING)打印自动停止
void startCalMonitor() {
  calMode = true;
  calMonitorLastMs = millis();     // 从"此刻"起等一个周期再打印,避开第一个采样还没落地的 0 值
  adcStartCalibration();
}

// 收到 's'/'S':刷新时间锚点并开始采集(必须先设时钟)。采样偏移以"所在整分"为基准
void startSensing() {
  if (!timeIsSet) {
    Serial.println("clock not set, send 'T YYYYMMDD HHMMSS' before 's'");
    return;
  }
  calMode = true;                  // 's' 触发的标定期间同样打印 raw/电压(和 'cal' 命令一致)
  calMonitorLastMs = millis();
  captureAnchor();

  char buf[24];
  fmtNow(buf, sizeof(buf));
  Serial.print("start @ ");
  Serial.println(buf);            // 's' 的确切时刻;基准 = 向下取整到整分

  adcStartCalibration();          // 启动 IO32 模拟通道的标定+采样
}

// 收到 'r'/'R':只复位事件计数 + 清空缓冲(不动时钟/时间基准/标定),保持当前启停状态
void zeroCounters() {
  adcResetRuntime();
  Serial.print("counter reset (ccc=1, buffer cleared)");
  if (timeIsSet) {
    char buf[24];
    fmtNow(buf, sizeof(buf));
    Serial.print(" @ ");
    Serial.print(buf);
  }
  Serial.println();
}

// 收到 'p'/'P':立即停止采集(缓冲里已有的会继续发完),同时退出 'cal' 调试模式
void stopSensing() {
  calMode = false;
  adcStop();
  Serial.println("stopped by command");
}

void printHelp() {
  Serial.println(FW_VERSION);
  Serial.println("commands:");
  Serial.println("  s/S   - start sensing (clock must be set first; prints raw/voltage during its 10s calibration)");
  Serial.println("  1~10  - after calibration, pick the zero(0%) offset level (-0.010V ~ -0.100V)");
  Serial.println("  p/P   - stop sensing (buffered samples keep flushing)");
  Serial.println("  r/R   - reset counter (ccc=1, clear buffer) + show time; clock untouched");
  Serial.println("  T ... - set clock: T YYYYMMDD HHMMSS  (e.g. T 20260827 140000)");
  Serial.println("  <space> - test shortcut: set clock to 2026-09-09 09:00:00");
  Serial.println("  cal   - debug: 10s zero-level calibration (no clock needed), prints raw/voltage only during that 10s window");
  Serial.println("  time  - print current clock (or 'not set')");
  Serial.println("  ping  - reply 'pong'");
  Serial.println("  ?     - print this list + current state");
  Serial.println("adc line: 'ccc tt.ttttS a.aaV vvvv xx%' = IO32 sample >1% (a.aaV/vvvv=voltage/raw for reference, xx%=amplitude)");
  Serial.println("'time:' base prints at each whole minute (sensing or not), ends with current adc voltage/raw/amplitude");

  Serial.print("state: ");
  Serial.println(adcPhase == AdcPhase::IDLE ? "stopped" : "sensing");
  if (timeIsSet) {
    char buf[24];
    fmtNow(buf, sizeof(buf));
    Serial.print("clock: ");
    Serial.println(buf);
  } else {
    Serial.println("clock: not set");
  }

  Serial.print("adc: ");
  switch (adcPhase) {
    case AdcPhase::IDLE:           Serial.println("idle"); break;
    case AdcPhase::CALIBRATING:    Serial.println("calibrating idle level..."); break;
    case AdcPhase::AWAITING_LEVEL: Serial.println("waiting for zero-level selection (send 1~10)"); break;
    case AdcPhase::RUNNING: {
      char b[40];
      snprintf(b, sizeof(b), "running (zero=%.3fV)", (double)adcZeroV);
      Serial.println(b);
      break;
    }
  }
}

// 纯数字字符串转挡位:1~ADC_ZERO_LEVEL_COUNT 合法返回 true,否则 false(不用 atoi,避免 "12abc" 这种误判)
bool parseLevelSelection(const char *s, int *level) {
  if (s[0] == '\0') return false;
  int v = 0;
  for (const char *p = s; *p; p++) {
    if (*p < '0' || *p > '9') return false;
    v = v * 10 + (*p - '0');
    if (v > ADC_ZERO_LEVEL_COUNT) return false;
  }
  if (v < 1) return false;
  *level = v;
  return true;
}

// 处理一行串口命令
void handleCmd(const char *s) {
  int lvl = 0;
  if      (!strcmp(s, "s") || !strcmp(s, "S")) startSensing();
  else if (!strcmp(s, "p") || !strcmp(s, "P")) stopSensing();
  else if (!strcmp(s, "r") || !strcmp(s, "R")) zeroCounters();
  else if (s[0] == 'T' && (s[1] == ' ' || s[1] == '\0')) setTimeCmd(s);
  else if (!strcmp(s, "time")) {
    if (timeIsSet) {
      char buf[24];
      fmtNow(buf, sizeof(buf));
      Serial.print("time: ");
      Serial.println(buf);
    } else {
      Serial.println("clock not set, send 'T YYYYMMDD HHMMSS'");
    }
  }
  else if (!strcmp(s, " ")) setTimeCmd("T 20260909 090000");  // 测试快捷键:空格 = 快速设成 2026-09-09 09:00:00
  else if (!strcmp(s, "cal"))                  startCalMonitor();  // 纯校准调试:10s 标定 + 持续打印 raw/电压
  else if (parseLevelSelection(s, &lvl))       selectZeroLevel(lvl);  // 标定后选 1~10 挡
  else if (!strcmp(s, "?"))                    printHelp();
  else if (!strcmp(s, "ping"))                 Serial.println("pong");
  else if (s[0] != '\0') { Serial.print("unknown cmd: "); Serial.println(s); }
}

// 非阻塞读取串口,按 \n 分行
void pollSerialInput() {
  static char buf[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[len] = '\0'; handleCmd(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

void setup() {
  Serial.setTxBufferSize(TX_BUF_BYTES);              // 必须在 begin 之前
  Serial.begin(115200);
  delay(50);
  setenv("TZ", "UTC0", 1);                           // 不做时区换算,输入几点就是几点
  tzset();
  Serial.println("I am starting... " FW_VERSION);    // 启动信息

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  esp_timer_create_args_t ledOffArgs = {};           // 振动指示灯的"到点关灯"一次性定时器
  ledOffArgs.callback = &ledOffTimerCallback;
  ledOffArgs.dispatch_method = ESP_TIMER_TASK;
  ledOffArgs.name = "led_off";
  esp_timer_create(&ledOffArgs, &ledOffTimer);

  pinMode(ADC_GND_GUARD_PIN, INPUT);                 // IO33 现场接了 GND,纯输入,不驱动/不上拉,避免和地线对冲
  analogReadResolution(12);                          // 0~4095
  analogSetPinAttenuation(ADC_PIN, ADC_11db);         // 满幅覆盖 0~3.3V

  Serial.println("ready, send 's' to start ('?' for help)");
}

void loop() {
  // ---- 心跳 LED:每 1s 翻转一次,loop 一旦卡住心跳就会停 ----
  // 振动指示灯改成硬件定时器驱动(见 adcTimerCallback/ledOffTimerCallback),不经过这里轮询;
  // 振动强制点亮期间(ledForcedOn)心跳只更新状态、不去碰引脚,等 ledOffTimer 到点自动交还控制权。
  static uint32_t ledLast = 0;
  uint32_t nowMs = millis();
  if (nowMs - ledLast >= LED_INTERVAL_MS) {
    ledLast = nowMs;
    heartbeatLedState = !heartbeatLedState;
    if (!ledForcedOn) digitalWrite(LED_PIN, heartbeatLedState);
  }

  // ---- 处理来自串口的命令(s / p / r / T / time / ping / ?) ----
  // 采集只在收到 'p'/'P' 时停止,没有自动停止
  pollSerialInput();

  // ---- 每到墙上时钟整分播报一次基准时间(需已设时钟;是否在采集都播报) ----
  // 这一行是后续 ADC 事件偏移的基准:事件绝对时刻 = 上一条 time: 的整分 + "sss.ssss S"
  // 仅在发送缓冲已清空时播报,保证上一分钟的事件都排在这行之前
  if (timeIsSet && adcTail == adcHead && (long)(time(NULL) / 60) != lastTimeMin) {
    emitTimeBase();
  }

  // ---- IO32 模拟通道:非阻塞发送一条 "ccc tt.ttttS a.aaV vvvv xx%" ----
  sendAdcLine();

  // ---- 校准调试打印:只在 10s 标定窗口内每 CAL_MONITOR_MS 打印一次 时间+raw/电压(3 位小数),标定一结束就自动停 ----
  // 用有符号比较:calMonitorLastMs 是在这次 loop 更靠后的地方(startSensing/startCalMonitor)
  // 用一次新的 millis() 设的,可能比这里最上面取的 nowMs 还晚一点点,无符号减法会下溢立刻触发一次误打印
  if (calMode && adcPhase == AdcPhase::CALIBRATING && (int32_t)(nowMs - calMonitorLastMs) >= (int32_t)CAL_MONITOR_MS) {
    calMonitorLastMs = nowMs;
    uint16_t mv = adcLastMillivolt;
    char tbuf[24];
    fmtNow(tbuf, sizeof(tbuf));      // 若时钟还没设过('cal' 不要求先设时钟),这里就是 1970 起算的默认时间
    char line[56];
    snprintf(line, sizeof(line), "%s  raw=%u v=%u.%03uV",
             tbuf, (unsigned)adcLastRaw, (unsigned)(mv / 1000), (unsigned)(mv % 1000));
    Serial.println(line);
  }
}
