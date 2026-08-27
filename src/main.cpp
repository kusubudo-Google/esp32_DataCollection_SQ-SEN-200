#include <Arduino.h>
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>

#define FW_VERSION "ver2.02.00"   // 固件版本(每次改动由 Claude 递增)

// ---------------- 配置 ----------------
constexpr int      LED_PIN         = 27;    // 心跳 LED,0.5 s 翻转一次,用来判断 MCU 是否活着
constexpr uint32_t LED_INTERVAL_MS = 500;
constexpr int      SENSOR_PIN      = 33;    // 振动传感器输入(IO33,仅输入脚,OK)
constexpr uint32_t TICK_US         = 100;   // 时间计数器分辨率 = 0.1 ms(时间戳单位 = 100µs)
constexpr uint32_t MIN_GAP_US      = 0;     // 去抖:两次下降沿最小间隔(µs),0 = 关闭
constexpr uint32_t BUF_SIZE        = 4096;  // 环形缓冲记录数(必须是 2 的幂)
constexpr uint32_t TX_BUF_BYTES    = 2048;  // 串口发送软缓冲

// ---------------- 环形缓冲(单生产者 ISR / 单消费者 loop,无需加锁) ----------------
// 只存相对时间(单位 100µs),次数由发送端自己数。uint32 在此单位下可存 ~5 天
static uint32_t ring[BUF_SIZE];
static volatile uint32_t head = 0;        // ISR 写
static volatile uint32_t tail = 0;        // loop 读
static volatile uint32_t dropped = 0;     // 缓冲满导致的丢失计数(正常应为 0)
static volatile uint32_t pulseCount = 0;  // ISR 侧总脉冲数(含被丢弃的),仅诊断用

static volatile bool     started    = false;  // true 表示已收到 's',正在计时采集
static volatile uint32_t t0_tick    = 0;      // 时间原点(收到 's' 的时刻,单位 = TICK_US)
static volatile uint32_t lastEdgeUs = 0;
static bool detached    = false;
static bool isrAttached = false;
static uint32_t sendIdx = 0;              // 发送端计数(= 输出里的 aa),只有 loop 碰

static bool     timeIsSet       = false;  // 是否已用 'T' 命令设过墙上时间
static uint32_t timeMsgAnchorMs = 0;      // 每分钟时间播报的基准(millis)

void IRAM_ATTR onFalling() {
  if (!started) return;                              // 's' 之前的脉冲忽略

  int64_t now = esp_timer_get_time();
  uint32_t us = (uint32_t)now;
  if (MIN_GAP_US && (us - lastEdgeUs) < MIN_GAP_US) { lastEdgeUs = us; return; }
  lastEdgeUs = us;

  pulseCount++;
  uint32_t t = (uint32_t)(now / TICK_US) - t0_tick;  // 相对 's' 的时间(单位 TICK_US)

  uint32_t next = (head + 1) & (BUF_SIZE - 1);
  if (next == tail) {
    dropped++;                                       // 缓冲满,丢弃
  } else {
    ring[head] = t;
    head = next;
  }
}

// 清空缓冲、把时间原点重置为现在、所有计数归零(不改动启停状态)
void resetState() {
  head = tail = 0;
  dropped = 0;
  pulseCount = 0;
  sendIdx = 0;
  int64_t now = esp_timer_get_time();
  lastEdgeUs = (uint32_t)now;
  t0_tick = (uint32_t)(now / TICK_US);               // 时间原点 = 现在
}

// 把当前墙上时间格式化成 "YYYY-MM-DD HH:MM:SS"
void fmtNow(char *out, size_t n) {
  time_t now = time(NULL);
  struct tm tm;
  localtime_r(&now, &tm);
  strftime(out, n, "%Y-%m-%d %H:%M:%S", &tm);
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

  char buf[24];
  fmtNow(buf, sizeof(buf));
  Serial.print("time set: ");
  Serial.println(buf);
}

// 收到 's'/'S':此刻即 t=0,复位并开始采集
void startSensing() {
  if (isrAttached) detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));
  resetState();
  detached = false;
  started = true;
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onFalling, FALLING);
  isrAttached = true;
  timeMsgAnchorMs = millis();                          // 每分钟播报从现在起算

  if (timeIsSet) {
    char buf[24];
    fmtNow(buf, sizeof(buf));
    Serial.print("start, t=0 @ ");
    Serial.println(buf);
  } else {
    Serial.println("start, t=0  (clock not set, send 'T YYYYMMDD HHMMSS')");
  }
}

// 收到 'r'/'R':计数器归零(时间原点 = 现在,次数清零,清空缓冲),保持当前启停状态
void zeroCounters() {
  if (started && isrAttached) {
    detachInterrupt(digitalPinToInterrupt(SENSOR_PIN));   // 归零期间挡住 ISR
    resetState();
    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onFalling, FALLING);
  } else {
    resetState();
  }
  Serial.println("counters zeroed, t=0ms");
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
  Serial.println("  s/S   - start sensing, t=0 at this moment");
  Serial.println("  p/P   - stop sensing (buffered pulses keep flushing)");
  Serial.println("  r/R   - zero counters (t=0 now, aa=0, clear buffer), keep run/stop state");
  Serial.println("  T ... - set clock: T YYYYMMDD HHMMSS  (e.g. T 20260827 140000)");
  Serial.println("  time  - print current clock (or 'not set')");
  Serial.println("  ping  - reply 'pong'");
  Serial.println("  ?     - print this list");
  Serial.println("after s/S: prints start time, then 'time:' line every 60s (if clock set)");
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

  // ---- 's' 之后每 60 秒播报一次绝对时间(需已设时钟) ----
  if (started && timeIsSet && (nowMs - timeMsgAnchorMs) >= 60000) {
    timeMsgAnchorMs += 60000;
    char buf[24];
    fmtNow(buf, sizeof(buf));
    Serial.print("time: ");
    Serial.println(buf);
  }

  // ---- 非阻塞发送:一次发一条,串口没空间就留在缓冲里下轮再发 ----
  if (tail != head) {
    uint32_t t = ring[tail];                    // 单位 100µs
    char line[32];
    // 格式 "aa ccc.cms":次数至少 2 位,时间保留 1 位小数(0.1ms 分辨率)
    int n = snprintf(line, sizeof(line), "%02lu %lu.%lums\n",
                     (unsigned long)sendIdx,
                     (unsigned long)(t / 10), (unsigned long)(t % 10));
    if (Serial.availableForWrite() >= n) {
      Serial.write((const uint8_t *)line, n);
      tail = (tail + 1) & (BUF_SIZE - 1);
      sendIdx++;
    }
  }
}
