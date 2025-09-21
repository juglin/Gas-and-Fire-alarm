#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/adc.h"
#include <DHT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int MQ2_AO_PIN = 34;
const int MQ2_DO_PIN = 27;
const int MQ6_AO_PIN = 35;
const int MQ6_DO_PIN = 14;
const int BUZZER_PIN = 13;
const int BUZZ_CH = 0;

#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const bool USE_DO_MQ2 = true;
const bool USE_DO_MQ6 = true;
const bool USE_BUZZER = true;

const unsigned long WARMUP_MS = 180000UL;
const unsigned long CALIB_MS = 20000UL;
const unsigned long SAMPLE_PERIOD_MS = 250UL;
const unsigned long REPORT_PERIOD_MS = 1000UL;
const unsigned long DHT_PERIOD_MS = 3000UL;

const float EMA_ALPHA = 0.05f; // หนืดขึ้นเพื่อลดแกว่ง

const float RISE_MQ2 = 1.25f, FALL_MQ2 = 1.12f;
const float RISE_MQ6 = 1.40f, FALL_MQ6 = 1.18f;

// หน่วงเวลา/ค้างสถานะ
const unsigned long ASSERT_HOLD_MS = 1500;
const unsigned long CLEAR_HOLD_MS = 7000;
const unsigned long MIN_ON_MS = 3000;

enum Stage
{
  WARMUP,
  CALIB,
  RUN
};
Stage stage = WARMUP;
unsigned long t_start = 0, t_lastSamp = 0, t_lastRep = 0, t_lastDHT = 0;

float ema2 = NAN, base2 = NAN;
float ema6 = NAN, base6 = NAN;

unsigned long t_lastRebase = 0;
const unsigned long SAFE_REBASE_INTERVAL_MS = 5UL * 60 * 1000;

volatile bool do2_trig = false, do6_trig = false;
volatile unsigned long do2_last = 0, do6_last = 0;
const unsigned long DO_DEBOUNCE_MS = 50;

const int BUZZ_PWM_FREQ = 2000;
const int BUZZ_DUTY = 128;
const unsigned long BUZZ_ON_MS = 120, BUZZ_OFF_MS = 120;
bool buzzer_on = false;
unsigned long t_buzz = 0;

float last_tempC = NAN;

// สถานะ + ตัวจับเวลา debounced
bool smoke = false, gas = false;
unsigned long gas_assert_start = 0, gas_clear_start = 0, gas_on_since = 0;
unsigned long smoke_assert_start = 0, smoke_clear_start = 0, smoke_on_since = 0;

int readADC(int pin) { return analogRead(pin); }
void emaUpdate(float x, float &e)
{
  if (isnan(e))
    e = x;
  else
    e = EMA_ALPHA * x + (1 - EMA_ALPHA) * e;
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
    if ((ledcRead(BUZZ_CH) > 0) && (now - t_buzz >= BUZZ_ON_MS))
    {
      ledcWrite(BUZZ_CH, 0);
      t_buzz = now;
    }
    else if ((ledcRead(BUZZ_CH) == 0) && (now - t_buzz >= BUZZ_OFF_MS))
    {
      ledcWrite(BUZZ_CH, BUZZ_DUTY);
      t_buzz = now;
    }
  }
}

void IRAM_ATTR isr_mq2()
{
  unsigned long now = millis();
  if (now - do2_last >= DO_DEBOUNCE_MS)
  {
    do2_trig = true;
    do2_last = now;
  }
}
void IRAM_ATTR isr_mq6()
{
  unsigned long now = millis();
  if (now - do6_last >= DO_DEBOUNCE_MS)
  {
    do6_trig = true;
    do6_last = now;
  }
}

void drawUI(bool smoke_, bool gas_, float tempC)
{
  display.clearDisplay();
  bool both = smoke_ && gas_;
  const char *msg = both ? "SMK+GAS" : (smoke_ ? "SMOKE" : (gas_ ? "GAS" : "SAFE"));
  if (smoke_ || gas_)
  {
    display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  else
  {
    display.setTextColor(SSD1306_WHITE);
  }
  display.setTextSize(both ? 2 : 3);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - (int)w) / 2, y = 8;
  display.setCursor(x, y);
  display.print(msg);
  display.setTextSize(1);
  char line[24];
  int y2 = 48;
  if (isnan(tempC))
    snprintf(line, sizeof(line), "Temp: --.- C");
  else
    snprintf(line, sizeof(line), "Temp: %.1f C", tempC);
  display.setCursor(8, y2);
  display.print(line);
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
    display.println("MQ2+MQ6+DHT11");
    display.display();
  }
  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_AO_PIN, ADC_11db);
  analogSetPinAttenuation(MQ6_AO_PIN, ADC_11db);
  pinMode(MQ2_AO_PIN, INPUT);
  pinMode(MQ6_AO_PIN, INPUT);

  if (USE_DO_MQ2)
  {
    pinMode(MQ2_DO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MQ2_DO_PIN), isr_mq2, CHANGE);
  }
  if (USE_DO_MQ6)
  {
    pinMode(MQ6_DO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MQ6_DO_PIN), isr_mq6, CHANGE);
  }

  if (USE_BUZZER)
  {
    ledcSetup(BUZZ_CH, BUZZ_PWM_FREQ, 8);
    ledcAttachPin(BUZZER_PIN, BUZZ_CH);
    buzzerStop();
  }

  dht.begin();
  t_start = t_lastSamp = t_lastRep = t_lastDHT = millis();
  t_lastRebase = millis();
  Serial.println("Stage: WARMUP...");
}

void loop()
{
  unsigned long now = millis();

  if (now - t_lastDHT >= DHT_PERIOD_MS)
  {
    t_lastDHT = now;
    float t = dht.readTemperature();
    if (!isnan(t))
      last_tempC = t;
  }

  if (now - t_lastSamp >= SAMPLE_PERIOD_MS)
  {
    t_lastSamp = now;
    int adc2 = readADC(MQ2_AO_PIN);
    int adc6 = readADC(MQ6_AO_PIN);
    emaUpdate((float)adc2, ema2);
    emaUpdate((float)adc6, ema6);

    switch (stage)
    {
    case WARMUP:
      if (now - t_start >= WARMUP_MS)
      {
        ema2 = NAN;
        ema6 = NAN;
        t_start = now;
        stage = CALIB;
        Serial.println("Stage: CALIB...");
      }
      break;
    case CALIB:
      if (now - t_start >= CALIB_MS)
      {
        base2 = ema2;
        if (base2 < 1)
          base2 = 1;
        base6 = ema6;
        if (base6 < 1)
          base6 = 1;
        stage = RUN;
        Serial.printf("Stage: RUN (base2=%.1f, base6=%.1f)\n", base2, base6);
      }
      break;
    case RUN:
      break;
    }
  }

  if (stage == RUN)
  {
    float r2 = (!isnan(ema2) && base2 > 0) ? (ema2 / base2) : 1.0f;
    float r6 = (!isnan(ema6) && base6 > 0) ? (ema6 / base6) : 1.0f;

    bool do2_low = false, do6_low = false;
    if (USE_DO_MQ2)
      do2_low = (digitalRead(MQ2_DO_PIN) == LOW);
    if (USE_DO_MQ6)
      do6_low = (digitalRead(MQ6_DO_PIN) == LOW);
    do2_trig = false;
    do6_trig = false;

    bool hazard6 = (r6 >= RISE_MQ6) || (USE_DO_MQ6 && do6_low);
    bool quiet6 = (r6 < FALL_MQ6) && (!USE_DO_MQ6 || !do6_low);

    if (!gas)
    {
      if (hazard6)
      {
        if (gas_assert_start == 0)
          gas_assert_start = now;
        if (now - gas_assert_start >= ASSERT_HOLD_MS)
        {
          gas = true;
          gas_on_since = now;
          gas_clear_start = 0;
        }
      }
      else
        gas_assert_start = 0;
    }
    else
    {
      bool min_on_ok = (now - gas_on_since >= MIN_ON_MS);
      if (quiet6 && min_on_ok)
      {
        if (gas_clear_start == 0)
          gas_clear_start = now;
        if (now - gas_clear_start >= CLEAR_HOLD_MS)
        {
          gas = false;
          gas_assert_start = 0;
        }
      }
      else
        gas_clear_start = 0;
    }

    bool hazard2 = (r2 >= RISE_MQ2) || (USE_DO_MQ2 && do2_low);
    bool quiet2 = (r2 < FALL_MQ2) && (!USE_DO_MQ2 || !do2_low);

    if (!smoke)
    {
      if (hazard2)
      {
        if (smoke_assert_start == 0)
          smoke_assert_start = now;
        if (now - smoke_assert_start >= ASSERT_HOLD_MS)
        {
          smoke = true;
          smoke_on_since = now;
          smoke_clear_start = 0;
        }
      }
      else
        smoke_assert_start = 0;
    }
    else
    {
      bool min_on_ok2 = (now - smoke_on_since >= MIN_ON_MS);
      if (quiet2 && min_on_ok2)
      {
        if (smoke_clear_start == 0)
          smoke_clear_start = now;
        if (now - smoke_clear_start >= CLEAR_HOLD_MS)
        {
          smoke = false;
          smoke_assert_start = 0;
        }
      }
      else
        smoke_clear_start = 0;
    }

    bool any_new = smoke || gas;
    static bool any_old = false;
    if (any_old && !any_new)
      buzzerStop();
    buzzerTick(any_new);
    any_old = any_new;

    if (!any_new)
    {
      if (now - t_lastRebase >= SAFE_REBASE_INTERVAL_MS)
      {
        base2 = 0.9f * base2 + 0.1f * ema2;
        base6 = 0.9f * base6 + 0.1f * ema6;
        t_lastRebase = now;
      }
    }
    else
      t_lastRebase = now;
  }
  else
  {
    buzzerStop();
    smoke = false;
    gas = false;
  }

  if (now - t_lastRep >= REPORT_PERIOD_MS)
  {
    t_lastRep = now;
    if (stage != RUN)
    {
      drawUI(false, false, last_tempC);
      Serial.println(stage == WARMUP ? "WARMUP..." : "CALIB...");
    }
    else
    {
      drawUI(smoke, gas, last_tempC);
      if (smoke && gas)
        Serial.println("SMK+GAS");
      else if (smoke)
        Serial.println("SMOKE");
      else if (gas)
        Serial.println("GAS");
      else
        Serial.println("SAFE");
    }
  }
}
