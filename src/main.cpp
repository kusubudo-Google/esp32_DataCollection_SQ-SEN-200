#include <Arduino.h>
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>

#define FW_VERSION "ver3.04.00"   // 固件版本(每次改动由 Claude 递增)

// ---------------- 配置 ----------------
constexpr int      LED_PIN         = 27;    // 心跳 LED,0.5 s 翻转一次,用来判断 MCU 是否活着
constexpr uint32_t LED_INTERVAL_MS = 500;
constexpr int      SENSOR_PIN      = 23;    // 振动传感器输入(IO23)
constexpr uint32_t TICK_US         = 100;   // 时间计数器分辨率 = 0.1 ms(时间戳单位 = 100µs)
constexpr uint32_t MIN_GAP_US      = 0;     // 去抖:两次下降沿最小间隔(µs),0 = 关闭
constexpr uint32_t BUF_SIZE        = 4096;  // 环形缓冲记录数(必须是 2 的幂)
constexpr uint32_t TX_BUF_BYTES    = 2048;  // 串口发送软缓冲

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

// 播报一行基准时间:"time: YYYY-MM-DD HH:MM:00",并记住这一分钟
void emitTimeBase() {
  time_t now = time(NULL);
  lastTimeMin = (long)(now / 60);
  char buf[24];
  fmtMinute(buf, sizeof(buf), now);
  Serial.print("time: ");
  Serial.println(buf);
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
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onFalling, FALLING);
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
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onFalling, FALLING);
  isrAttached = true;

  char buf[24];
  fmtNow(buf, sizeof(buf));
  Serial.print("start @ ");
  Serial.println(buf);            // 's' 的确切时刻;基准 = 向下取整到整分
}

// 收到 'r'/'R':只复位脉冲计数 + 清空缓冲(不动时钟/时间基准),保持当前启停状态
void zeroCounters() {
  if (started && isrAttached) {
    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));   // 清缓冲期间挡住 ISR
    clearBuffer();
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onFalling, FALLING);
  } else {
    clearBuffer();
  }
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
  Serial.println("'time:' base prints at each whole minute (sensing or not)");

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

  pinMode(SENSOR_PIN, INPUT);                        // 传感器为推挽输出,不需要内部上拉
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onFalling, FALLING);
  isrAttached = true;
  started = false;                                   // 等 's'/'S' 才开始计时
  Serial.println("ready, send 's' to start ('?' for help)");
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
  // 仅在发送缓冲已清空时播报,保证上一分钟的脉冲都排在这行之前
  if (timeIsSet && tail == head && (long)(time(NULL) / 60) != lastTimeMin) {
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
}
