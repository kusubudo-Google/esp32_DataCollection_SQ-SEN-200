#include <Arduino.h>

const int LED_PIN = 27;
const uint64_t TOGGLE_INTERVAL_US = 500000;  // 0.5s

hw_timer_t *timer = nullptr;
volatile bool ledState = false;
volatile uint32_t toggleCount = 0;  // 未处理的 toggle 次数
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
  toggleCount++;  // 只做标记，串口打印放到 loop() 里
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);

  // timer 0, prescaler 80 -> 80MHz/80 = 1MHz (1 tick = 1us), count up
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TOGGLE_INTERVAL_US, true);  // autoreload
  timerAlarmEnable(timer);
}

void loop() {
  static uint32_t cnt = 0;

  // 取出并清零 ISR 累计的 toggle 次数（读+清零作为一个整体）
  portENTER_CRITICAL(&timerMux);
  uint32_t pending = toggleCount;
  toggleCount = 0;
  portEXIT_CRITICAL(&timerMux);

  // 每次 toggle 都发一行，即使 loop 慢了积压也不漏
  for (uint32_t i = 0; i < pending; i++) {
    Serial.printf("I am uart,cnt=%u\n", ++cnt);
  }
}
