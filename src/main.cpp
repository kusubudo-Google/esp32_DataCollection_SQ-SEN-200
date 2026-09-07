#include <Arduino.h>
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>

/* ================= IO32 模拟振动通道(ADC)需求规格 =====================
 * 本节是 IO32 模拟振动检测功能的规格存档;以后这部分需求有变动,
 * 必须同步改这段注释,不要只改代码。
 *
 * 1) 串口发 's'/'S' 启动 ADC 采样(与数字通道共用同一个命令),满幅 0V~3.3V。
 * 2) 空闲(无振动)= 高电平 → 定义为 0%;持续大振动 = 接近 0V → 定义为 100%。
 * 3) 0% 基线标定:'s' 之后先连续采样 ADC_CAL_MS(10 s),统计均值 mean 和
 *    峰峰值噪声半幅 noiseAmp = (max-min)/2,
 *      threshold(0%) = mean - noiseAmp - ADC_IDLE_MARGIN_V
 *    电压 > threshold 都记为 0%(现场实测:mean≈3.188V,noiseAmp≈0.10V,
 *    ADC_IDLE_MARGIN_V=0.005V → threshold≈3.083V;noiseAmp 是标定时实测出来的,
 *    不是写死的值,所以噪声大小变化不用改代码)。
 *    另设一个绝对期望值 ADC_NOMINAL_IDLE_V;若标定出的 mean 偏离它超过
 *    ADC_DRIFT_WARN_V(例:期望 3.18V,标定出 3.12V),判定为"可用但异常",
 *    通过串口发一行英文 WARNING 提示。
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
 *    esp_timer 硬上限在 120~135µs 之间,超过就必炸 task_wdt 重启,50µs 不可行,
 *    默认改用留有余量的 200µs/5kHz,见 ADC_SAMPLE_US 的注释)。空闲(≤1%)不输出,
 *    不占用串口带宽。
 * 6) 'T' 设置时钟、每整分播报一次 "time: ..." 的逻辑不变;只在这行末尾追加
 *    当前(最新一次采样)的电压/原始值/幅度,例如
 *    "time: 2026-09-07 14:23:00 3.19V 3958 0%"。
 *
 * IO33 与 IO32 共用同一个传感器插头,现场已把 IO33 接到 GND,固件把它设为
 * 纯输入(不驱动、不加内部上拉),避免和外部 GND 对冲。
 * ===================================================================== */

#define FW_VERSION "ver3.08.00"   // 固件版本(每次改动由 Claude 递增)

// ---------------- 配置 ----------------
#define SENSOR_ENABLED 0             // IO13 振动检测开关:1=正常挂中断采集,0=禁用中断(仅内部上拉,不响应任何脉冲)
                                      // 现场 IO13 悬空/未接传感器时设为 0,接好传感器后改回 1 即可恢复采集
constexpr int      LED_PIN         = 27;    // 心跳 LED,0.5 s 翻转一次,用来判断 MCU 是否活着
constexpr uint32_t LED_INTERVAL_MS = 500;
constexpr int      SENSOR_PIN      = 13;    // 振动传感器输入(IO13)
constexpr uint32_t TICK_US         = 100;   // 时间计数器分辨率 = 0.1 ms(时间戳单位 = 100µs)
constexpr uint32_t MIN_GAP_US      = 0;     // 去抖:两次下降沿最小间隔(µs),0 = 关闭
constexpr uint32_t BUF_SIZE        = 4096;  // 环形缓冲记录数(必须是 2 的幂)
constexpr uint32_t TX_BUF_BYTES    = 2048;  // 串口发送软缓冲

// ---------------- ADC 模拟振动通道配置(IO32) ----------------
constexpr int      ADC_PIN            = 32;      // 模拟振动传感器输入(IO32, ADC1_CH4)
constexpr int      ADC_GND_GUARD_PIN  = 33;      // 现场接了 GND 的邻脚(同一插头),只设输入,不驱动/不上拉
constexpr uint32_t ADC_SAMPLE_US      = 200;     // 采样周期 200µs = 5kHz。esp_timer 回调跑在专用任务里,
                                                  // 实机二分法测过硬上限:100/120µs 必炸 task_wdt(CPU 被
                                                  // 占满,IDLE0 喂不上看门狗),135µs 起才稳,即硬上限在
                                                  // 120~135µs(约 7.4~8.3kHz)之间,且卡得很死、没有余量。
                                                  // 5kHz 离那条线还有 ~1.5 倍余量,200/250/500µs/1kHz 都
                                                  // 实测跑满 10s 标定无异常,可按需在这几档之间调整
constexpr uint32_t ADC_CAL_MS         = 10000;   // 's' 后先花 10s 标定"空闲高电平=0%"基线
constexpr float    ADC_IDLE_MARGIN_V  = 0.005f;  // 0% 基线 = 标定均值 - 噪声半幅 - 该余量
constexpr float    ADC_LOW_FLOOR_V    = 0.100f;  // 低于此电压(100mV)固定记为 100%
constexpr float    ADC_NOMINAL_IDLE_V = 3.188f;  // 期望的空闲电压(现场实测值),标定值偏离它超过下面阈值就报警
constexpr float    ADC_DRIFT_WARN_V   = 0.05f;   // 允许的漂移范围(V)
constexpr uint32_t ADC_BUF_SIZE       = 2048;    // ADC 事件发送环形缓冲(2 的幂),吸收一次振动事件的突发采样

// ---------------- 环形缓冲(单生产者 ISR / 单消费者 loop,无需加锁) ----------------
// 只存"距所在整分的偏移"(单位 100µs,0~599999),次数由发送端自己数
static uint32_t ring[BUF_SIZE];
static volatile uint32_t head = 0;        // ISR 写
static volatile uint32_t tail = 0;        // loop 读
static volatile uint32_t dropped = 0;     // 缓冲满导致的丢失计数(正常应为 0)
static volatile uint32_t pulseCount = 0;  // ISR 侧总脉冲数(含被丢弃的),仅诊断用

static volatile bool    started       = false;  // true 表示已收到 's',正在采集
static volatile int64_t anchorEspUs   = 0;      // 锚点:esp_timer 读数
static volatile int64_t anchorEpochUs = 0;      // 锚点:同一时刻的墙上时间(µs since epoch)
static volatile uint32_t lastEdgeUs   = 0;
static bool detached    = false;
static bool isrAttached = false;
static uint32_t sendIdx = 1;              // 发送端计数(= 输出里的 aa,从 1 开始),只有 loop 碰

static bool timeIsSet    = false;  // 是否已用 'T' 命令设过墙上时间
static long lastTimeMin  = -1;     // 上次播报的"墙上分钟号"(epoch/60),用于整分触发一次

// 记录 esp_timer 与墙上时间的对应关系,ISR 靠它推算每个脉冲的绝对时刻
void captureAnchor() {
  int64_t esp = esp_timer_get_time();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  anchorEspUs   = esp;
  anchorEpochUs = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

void IRAM_ATTR onFalling() {
  if (!started) return;                              // 's' 之前的脉冲忽略

  int64_t now = esp_timer_get_time();
  uint32_t us = (uint32_t)now;
  if (MIN_GAP_US && (us - lastEdgeUs) < MIN_GAP_US) { lastEdgeUs = us; return; }
  lastEdgeUs = us;

  pulseCount++;
  int64_t epochUs = anchorEpochUs + (now - anchorEspUs);       // 该脉冲的绝对时刻(µs)
  uint32_t t = (uint32_t)((epochUs % 60000000LL) / TICK_US);   // 距所在整分的偏移(单位 TICK_US)

  uint32_t next = (head + 1) & (BUF_SIZE - 1);
  if (next == tail) {
    dropped++;                                       // 缓冲满,丢弃
  } else {
    ring[head] = t;
    head = next;
  }
}

// 统一的中断挂载入口:SENSOR_ENABLED=0 时永远不真正挂中断,isrAttached 也保持 false,
// 这样所有依赖 isrAttached 的分支(setTimeCmd/zeroCounters/stopSensing)都会自动走"不碰中断"的路径
void sensorAttachInterrupt() {
#if SENSOR_ENABLED
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onFalling, FALLING);
  isrAttached = true;
#else
  isrAttached = false;
#endif
}

// 只清空缓冲和计数(不动时间锚点)
void clearBuffer() {
  head = tail = 0;
  dropped = 0;
  pulseCount = 0;
  sendIdx = 1;                                       // aa 从 1 开始
  lastEdgeUs = (uint32_t)esp_timer_get_time();
}

// clearBuffer + 刷新时间锚点(用于 's' 开始采集)
void resetState() {
  clearBuffer();
  captureAnchor();
}

// ---------------- ADC 模拟振动通道(IO32,esp_timer 定时采样,单独一套环形缓冲) ----------------
enum class AdcPhase : uint8_t { IDLE, CALIBRATING, RUNNING };

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
static volatile uint16_t adcLastCentivolt = 0;   // 最新一次采样的电压*100(同上)

static esp_timer_handle_t adcTimer = nullptr;
static int64_t  adcCalStartUs = 0;
static double   adcCalSum     = 0;
static float    adcCalMin     = 0;
static float    adcCalMax     = 0;
static uint32_t adcCalCount   = 0;

// 10s 标定窗口结束:算出 0% 基线,顺带检查是否偏离期望值太多
void adcFinishCalibration() {
  float mean = (adcCalCount > 0) ? (float)(adcCalSum / adcCalCount) : 0.0f;
  float noiseAmp = (adcCalMax - adcCalMin) / 2.0f;
  float zero = mean - noiseAmp - ADC_IDLE_MARGIN_V;
  if (zero < ADC_LOW_FLOOR_V + 0.05f) zero = ADC_LOW_FLOOR_V + 0.05f;  // 兜底,避免分母太小/为负
  adcZeroV = zero;
  adcPhase = AdcPhase::RUNNING;

  char line[112];
  snprintf(line, sizeof(line), "ADC calibrated: idle=%.3fV noise=+-%.3fV zero(0%%)=%.3fV (n=%lu)",
           mean, noiseAmp, zero, (unsigned long)adcCalCount);
  Serial.println(line);

  float drift = mean - ADC_NOMINAL_IDLE_V;
  float absDrift = drift < 0 ? -drift : drift;
  if (absDrift > ADC_DRIFT_WARN_V) {
    char warn[144];
    snprintf(warn, sizeof(warn),
             "WARNING: ADC idle level drifted to %.3fV (nominal %.3fV, delta %.3fV) - still usable but check sensor/wiring",
             mean, ADC_NOMINAL_IDLE_V, drift);
    Serial.println(warn);
  }
}

// esp_timer 周期回调(任务上下文,非真正 ISR,可以放心调 analogRead/Serial):每 ADC_SAMPLE_US 跑一次
void adcTimerCallback(void *) {
  int      raw = analogRead(ADC_PIN);              // 原始采样值 0~4095,单独读一次(用于调试展示)
  float    v   = analogReadMilliVolts(ADC_PIN) / 1000.0f;  // 校准后电压,百分比计算仍然只认这个

  if (adcPhase == AdcPhase::CALIBRATING) {
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

  if (pct > 1) {                                          // 只有 >1% 才算一次事件,进缓冲等待发送
    int64_t now = esp_timer_get_time();
    int64_t epochUs = anchorEpochUs + (now - anchorEspUs); // 复用数字通道同一套时钟锚点
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

// 把 epoch 秒向下取整到整分,格式化成 "YYYY-MM-DD HH:MM:00"(脉冲偏移的基准)
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

  // 刷新 ISR 用的时间锚点;若正在采集,短暂 detach 避免 64 位撕裂读
  if (started && isrAttached) {
    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
    captureAnchor();
    sensorAttachInterrupt();
  } else {
    captureAnchor();
  }

  char buf[24];
  fmtNow(buf, sizeof(buf));
  Serial.print("time set: ");
  Serial.println(buf);
}

// 收到 's'/'S':复位并开始采集(必须先设时钟)。脉冲偏移以"所在整分"为基准
void startSensing() {
  if (!timeIsSet) {
    Serial.println("clock not set, send 'T YYYYMMDD HHMMSS' before 's'");
    return;
  }
  if (isrAttached) detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
  resetState();
  detached = false;
  started = true;
  sensorAttachInterrupt();

  char buf[24];
  fmtNow(buf, sizeof(buf));
  Serial.print("start @ ");
  Serial.println(buf);            // 's' 的确切时刻;基准 = 向下取整到整分
#if !SENSOR_ENABLED
  Serial.println("note: IO13 sensing DISABLED (SENSOR_ENABLED=0), no pulses will be captured");
#endif
  adcStartCalibration();          // 同一个 's' 顺带启动 IO32 模拟通道的标定+采样
}

// 收到 'r'/'R':只复位脉冲计数 + 清空缓冲(不动时钟/时间基准),保持当前启停状态
void zeroCounters() {
  if (started && isrAttached) {
    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));   // 清缓冲期间挡住 ISR
    clearBuffer();
    sensorAttachInterrupt();
  } else {
    clearBuffer();
  }
  adcResetRuntime();             // ADC 事件计数/缓冲同步复位(不重新标定)
  Serial.print("pulse counter reset (aa=1, buffer cleared)");
  if (timeIsSet) {
    char buf[24];
    fmtNow(buf, sizeof(buf));
    Serial.print(" @ ");
    Serial.print(buf);
  }
  Serial.println();
}

// 收到 'p'/'P':立即停止采集(缓冲里已有的会继续发完)
void stopSensing() {
  started = false;
  if (isrAttached && !detached) {
    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
    detached = true;
    isrAttached = false;
  }
  adcStop();
  Serial.println("stopped by command");
}

void printHelp() {
  Serial.println("SQ-SEN-200 vibration logger  " FW_VERSION);
  Serial.println("commands:");
  Serial.println("  s/S   - start sensing (clock must be set first)");
  Serial.println("  p/P   - stop sensing (buffered pulses keep flushing)");
  Serial.println("  r/R   - reset pulse counter (aa=1, clear buffer) + show time; clock untouched");
  Serial.println("  T ... - set clock: T YYYYMMDD HHMMSS  (e.g. T 20260827 140000)");
  Serial.println("  time  - print current clock (or 'not set')");
  Serial.println("  ping  - reply 'pong'");
  Serial.println("  ?     - print this list + current state");
  Serial.println("pulse line: 'aa sss.ssss s' = offset from the preceding 'time: HH:MM:00' base");
  Serial.println("adc line:   'ccc tt.ttttS a.aaV vvvv xx%' = IO32 sample >1% (a.aaV/vvvv=voltage/raw for reference, xx%=amplitude)");
  Serial.println("'time:' base prints at each whole minute (sensing or not), ends with current adc voltage/raw/amplitude");

  Serial.print("state: ");
  Serial.println(started ? "sensing" : "stopped");
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
    case AdcPhase::IDLE:        Serial.println("idle"); break;
    case AdcPhase::CALIBRATING: Serial.println("calibrating idle level..."); break;
    case AdcPhase::RUNNING: {
      char b[40];
      snprintf(b, sizeof(b), "running (zero=%.3fV)", (double)adcZeroV);
      Serial.println(b);
      break;
    }
  }
}

// 处理一行串口命令
void handleCmd(const char *s) {
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

  pinMode(SENSOR_PIN, INPUT_PULLUP);                 // IO13 当前悬空/传感器未接,开内部上拉防止误触发
  sensorAttachInterrupt();
  started = false;                                   // 等 's'/'S' 才开始计时
#if SENSOR_ENABLED
  Serial.println("ready, send 's' to start ('?' for help)");
#else
  Serial.println("IO13 sensing DISABLED (SENSOR_ENABLED=0, pull-up only) - edit SENSOR_ENABLED and reflash to re-enable");
#endif

  pinMode(ADC_GND_GUARD_PIN, INPUT);                 // IO33 现场接了 GND,纯输入,不驱动/不上拉,避免和地线对冲
  analogReadResolution(12);                          // 0~4095
  analogSetPinAttenuation(ADC_PIN, ADC_11db);         // 满幅覆盖 0~3.3V
}

void loop() {
  // ---- 心跳 LED:每 0.5 s 翻转,loop 一旦卡住 LED 就会停 ----
  static uint32_t ledLast = 0;
  static bool ledState = false;
  uint32_t nowMs = millis();
  if (nowMs - ledLast >= LED_INTERVAL_MS) {
    ledLast = nowMs;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  // ---- 处理来自串口的命令(s / p / r / T / ping) ----
  // 采集只在收到 'p'/'P' 时停止,没有自动停止
  pollSerialInput();

  // ---- 每到墙上时钟整分播报一次基准时间(需已设时钟;是否在采集都播报) ----
  // 这一行是后续脉冲偏移的基准:pulse 绝对时刻 = 上一条 time: 的整分 + "sss.ssss s"
  // 仅在两路发送缓冲都已清空时播报,保证上一分钟的脉冲/ADC 事件都排在这行之前
  if (timeIsSet && tail == head && adcTail == adcHead && (long)(time(NULL) / 60) != lastTimeMin) {
    emitTimeBase();
  }

  // ---- 非阻塞发送:一次发一条,串口没空间就留在缓冲里下轮再发 ----
  if (tail != head) {
    uint32_t t = ring[tail];                    // 距所在整分的偏移,单位 100µs(0~599999)
    char line[32];
    // 格式 "aa sss.ssss s":次数至少 2 位,偏移 = 距最近 time: 的秒数,保留 4 位小数(0.1ms)
    int n = snprintf(line, sizeof(line), "%02lu %lu.%04lu s\n",
                     (unsigned long)sendIdx,
                     (unsigned long)(t / 10000), (unsigned long)(t % 10000));
    if (Serial.availableForWrite() >= n) {
      Serial.write((const uint8_t *)line, n);
      tail = (tail + 1) & (BUF_SIZE - 1);
      sendIdx++;
    }
  }

  // ---- IO32 模拟通道:非阻塞发送一条 "ccc tt.ttttS xx%" ----
  sendAdcLine();
}
