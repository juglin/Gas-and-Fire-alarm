#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/adc.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int MQ2_AO_PIN = 34;
const int MQ2_DO_PIN = 27;
const int BUZZER_PIN = 13;
const int BUZZ_CH = 0;

const bool USE_DO = true;
const bool USE_BUZZER = true;

const unsigned long WARMUP_MS = 60000UL;
const unsigned long CALIB_MS = 10000UL;
const unsigned long SAMPLE_PERIOD_MS = 200UL;
const unsigned long REPORT_PERIOD_MS = 1000UL;

const float EMA_ALPHA = 0.15f;
const float RISE_RATIO = 1.10f;
const float FALL_RATIO = 1.05f;

enum Stage
{
  WARMUP,
  CALIB,
  RUN
};
Stage stage = WARMUP;
unsigned long t_start = 0, t_lastSamp = 0, t_lastRep = 0;

float ema = NAN;
float baseline = NAN;

volatile bool do_triggered = false;
volatile unsigned long do_last_irq_ms = 0;
const unsigned long DO_DEBOUNCE_MS = 50;

const int BUZZ_PWM_FREQ = 2000;
const int BUZZ_DUTY = 128;
const unsigned long BUZZ_BEEP_ON_MS = 120;
const unsigned long BUZZ_BEEP_OFF_MS = 120;
bool buzzer_on = false;
unsigned long t_buzz = 0;

int readADC()
{
  return analogRead(MQ2_AO_PIN);
}

void emaUpdate(float x)
{
  if (isnan(ema))
    ema = x;
  else
    ema = EMA_ALPHA * x + (1 - EMA_ALPHA) * ema;
}

void buzzerStop()
{
  if (!USE_BUZZER)
    return;
  ledcWrite(BUZZ_CH, 0);
  buzzer_on = false;
}

void buzzerTick(bool alarm)
{
  if (!USE_BUZZER)
    return;
  if (!alarm)
  {
    buzzerStop();
    return;
  }
  unsigned long now = millis();
  if (!buzzer_on)
  {
    ledcWrite(BUZZ_CH, BUZZ_DUTY);
    buzzer_on = true;
    t_buzz = now;
  }
  else
  {
    if ((ledcRead(BUZZ_CH) > 0) && (now - t_buzz >= BUZZ_BEEP_ON_MS))
    {
      ledcWrite(BUZZ_CH, 0);
      t_buzz = now;
    }
    else if ((ledcRead(BUZZ_CH) == 0) && (now - t_buzz >= BUZZ_BEEP_OFF_MS))
    {
      ledcWrite(BUZZ_CH, BUZZ_DUTY);
      t_buzz = now;
    }
  }
}

void IRAM_ATTR do_isr()
{
  unsigned long now = millis();
  if (now - do_last_irq_ms >= DO_DEBOUNCE_MS)
  {
    do_triggered = true;
    do_last_irq_ms = now;
  }
}

void drawOLED(bool smoke)
{
  display.clearDisplay();
  if (smoke)
  {
    display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  else
  {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setTextSize(3);
  const char *msg = smoke ? "SMOKE" : "SAFE";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - (int)w) / 2;
  int y = (SCREEN_HEIGHT - (int)h) / 2;
  display.setCursor(x, y);
  display.print(msg);
  display.display();
}

void setup()
{
  Serial.begin(9600);
  delay(100);
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println("OLED init failed!");
  }
  else
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("MQ-2 Smoke Detector");
    display.setCursor(0, 10);
    display.println("Initializing...");
    display.display();
  }
  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_AO_PIN, ADC_11db);
  pinMode(MQ2_AO_PIN, INPUT);
  if (USE_DO)
  {
    pinMode(MQ2_DO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MQ2_DO_PIN), do_isr, CHANGE);
  }
  if (USE_BUZZER)
  {
    ledcSetup(BUZZ_CH, BUZZ_PWM_FREQ, 8);
    ledcAttachPin(BUZZER_PIN, BUZZ_CH);
    buzzerStop();
  }
  t_start = t_lastSamp = t_lastRep = millis();
  Serial.println("\n[MQ2] ESP32 + OLED (SAFE/SMOKE only)");
  Serial.println("Stage: WARMUP...");
}

void loop()
{
  unsigned long now = millis();
  if (now - t_lastSamp >= SAMPLE_PERIOD_MS)
  {
    t_lastSamp = now;
    int adc = readADC();
    emaUpdate((float)adc);
    switch (stage)
    {
    case WARMUP:
      if (now - t_start >= WARMUP_MS)
      {
        ema = NAN;
        t_start = now;
        stage = CALIB;
        Serial.println("Stage: CALIBRATING baseline...");
      }
      break;
    case CALIB:
      if (now - t_start >= CALIB_MS)
      {
        baseline = ema;
        if (baseline < 1)
          baseline = 1;
        stage = RUN;
        Serial.print("Stage: RUN (baseline=");
        Serial.print(baseline, 1);
        Serial.println(")");
      }
      break;
    case RUN:
      break;
    }
  }

  static bool smoke = false;
  bool smoke_by_ratio = false, smoke_by_do = false;
  float ratio = NAN;
  int do_val = -1;

  if (stage == RUN)
  {
    if (!isnan(ema) && baseline > 0)
    {
      ratio = ema / baseline;
      if (!smoke && ratio >= RISE_RATIO)
        smoke_by_ratio = true;
      else if (smoke && ratio >= FALL_RATIO)
        smoke_by_ratio = true;
    }
    if (USE_DO)
    {
      do_val = digitalRead(MQ2_DO_PIN);
      smoke_by_do = (do_val == LOW);
      if (do_triggered)
        do_triggered = false;
    }
    bool new_smoke = smoke_by_ratio || (USE_DO && smoke_by_do);
    if (smoke && !new_smoke)
      buzzerStop();
    smoke = new_smoke;
    buzzerTick(smoke);
  }
  else
  {
    buzzerStop();
  }

  if (now - t_lastRep >= REPORT_PERIOD_MS)
  {
    t_lastRep = now;
    if (stage == WARMUP)
    {
      Serial.printf("[WARMUP] %lus/%lus\n", (now - t_start) / 1000, WARMUP_MS / 1000);
      drawOLED(false);
    }
    else if (stage == CALIB)
    {
      Serial.printf("[CALIB]  %lus/%lus  ADC(ema)=%.1f\n", (now - t_start) / 1000, CALIB_MS / 1000, ema);
      drawOLED(false);
    }
    else
    {
      Serial.println(smoke ? "[RUN] SMOKE" : "[RUN] SAFE");
      drawOLED(smoke);
    }
  }
}
