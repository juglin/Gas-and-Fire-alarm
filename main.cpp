
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

/* -------- OLED -------- */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* -------- Pins (แก้ตามบอร์ดจริง) -------- */
const int BUZZER_PIN = 13;
const int BUZZ_CH = 0;

// MQ-2 (ควัน)
const int MQ2_AO_PIN = 34;
const int MQ2_DO_PIN = 27;

// MQ-6 (แก๊ส)
const int MQ6_AO_PIN = 35;
const int MQ6_DO_PIN = 14;

// DHT11 (Temp)
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

/* -------- Tunables -------- */
const unsigned long WARMUP_MS = 30000UL;
const unsigned long CALIB_MS = 10000UL;

const float EMA_ALPHA_MQ2 = 0.15f;
const float EMA_ALPHA_MQ6 = 0.12f;

const float RATIO_ON_MQ2 = 1.25f;
const float RATIO_OFF_MQ2 = 1.12f;
const int ABS_ON_MQ2 = 150;

const float RATIO_ON_MQ6 = 1.20f;
const float RATIO_OFF_MQ6 = 1.10f;
const int ABS_ON_MQ6 = 120;

// DHT read interval
const unsigned long DHT_READ_INTERVAL_MS = 1000UL;

// DO active logic
const bool DO_ACTIVE_HIGH = true;

/* -------- ADC/PPM Conversion (สำคัญ) -------- */
const float VREF_ADC = 3.30f;  // ช่วงอ้างอิง ADC (ประมาณเมื่อใช้ ADC_11db)
const float VC_SENSOR = 5.00f; // แรงดันจ่ายเซนเซอร์ MQ-6 (เช็คจริงบนบอร์ด)
const float RL_OHM = 5000.0f;  // ตัวต้านทานโหลดของโมดูล MQ-6 (เช็คสกรีนบนบอร์ด)

// เราจะหา R0 (Rs ในอากาศสะอาด) ตอนจบขั้น CALIBRATE
float R0_mq6 = 0.0f;

// ฟิตกราฟ Rs/R0 = A * ppm^(-B) จาก "2 จุด" บนกราฟ datasheet ของแก๊สเป้าหมาย (ที่นี่ใช้ LPG เป็นตัวอย่าง)
struct MQCurve
{
  float A;
  float B;
};
MQCurve curve_mq6_LPG;

const float LPG_PPM_1 = 200.0f;
const float LPG_RATIO_1 = 3.2f; // Rs/R0 @ ~200 ppm (ตัวอย่าง)
const float LPG_PPM_2 = 1000.0f;
const float LPG_RATIO_2 = 0.8f; // Rs/R0 @ ~1000 ppm (ตัวอย่าง)

/* -------- State -------- */
enum Stage
{
  WARMUP,
  CALIBRATE,
  RUN
};
Stage stage = WARMUP;
unsigned long tStageStart = 0;

uint64_t calibSumMQ2 = 0;
uint32_t calibN_MQ2 = 0;
uint64_t calibSumMQ6 = 0;
uint32_t calibN_MQ6 = 0;

float baselineMQ2 = 0.0f, baselineMQ6 = 0.0f;
float emaMQ2 = -1.0f, emaMQ6 = -1.0f;

bool smoke = false, gasLeak = false;

// DHT cache
float dhtTempC = NAN;
unsigned long nextDhtReadAt = 0;

/* -------- Interrupt (DO) -------- */
volatile bool mq2_irq = false;
volatile bool mq6_irq = false;
volatile unsigned long mq2_irq_time = 0;
volatile unsigned long mq6_irq_time = 0;
const unsigned long DO_DEBOUNCE_MS = 20;

void IRAM_ATTR mq2_isr()
{
  mq2_irq = true;
  mq2_irq_time = millis();
}
void IRAM_ATTR mq6_isr()
{
  mq6_irq = true;
  mq6_irq_time = millis();
}

/* -------- Buzzer (non-blocking) -------- */
struct BuzzerState
{
  bool active = false;
  int freq = 1000;
  bool toneOn = false;
  unsigned long nextToggleAt = 0;
  unsigned long onMs = 120, offMs = 40;
} buz;

static inline void buzStart(int freq, unsigned long onMs = 120, unsigned long offMs = 40)
{
  buz.active = true;
  buz.freq = freq;
  buz.onMs = onMs;
  buz.offMs = offMs;
  buz.toneOn = false;
  buz.nextToggleAt = 0;
}
static inline void buzStop()
{
  buz.active = false;
  ledcWriteTone(BUZZ_CH, 0);
  buz.toneOn = false;
}
static inline void buzUpdate(unsigned long now)
{
  if (!buz.active)
    return;
  if (buz.nextToggleAt == 0 || now >= buz.nextToggleAt)
  {
    if (buz.toneOn)
    {
      ledcWriteTone(BUZZ_CH, 0);
      buz.toneOn = false;
      buz.nextToggleAt = now + buz.offMs;
    }
    else
    {
      ledcWriteTone(BUZZ_CH, buz.freq);
      buz.toneOn = true;
      buz.nextToggleAt = now + buz.onMs;
    }
  }
}

/* -------- Helpers -------- */

// map ระดับเหนือ baseline -> ความถี่เสียงเตือน
int levelToFreqFromBaseline(float value, float base)
{
  float span = value - base;
  if (span < 0.0f)
    span = 0.0f;
  int f = 800 + (int)(span * 2.5f);
  if (f > 2600)
    f = 2600;
  return f;
}

// ===== PPM conversion helpers =====
float mq_compute_Rs(float vout, float vc, float RL_ohm)
{
  if (vout < 0.001f)
    vout = 0.001f;
  return RL_ohm * (vc - vout) / vout;
}
MQCurve mq_fit_curve(float ppm1, float ratio1, float ppm2, float ratio2)
{
  // log10(ratio) = log10(A) - B*log10(ppm)
  float x1 = log10f(ppm1), y1 = log10f(ratio1);
  float x2 = log10f(ppm2), y2 = log10f(ratio2);
  float B = (y1 - y2) / (x2 - x1);
  float logA = y1 + B * x1;
  MQCurve c;
  c.A = powf(10.0f, logA);
  c.B = B;
  return c;
}
float mq_ppm_from_ratio(float ratio, MQCurve c)
{
  if (ratio <= 0.0f)
    ratio = 1e-6f;
  // ratio = A * ppm^-B -> ppm = (ratio/A)^(-1/B)
  return powf(ratio / c.A, -1.0f / c.B);
}

// OLED: 4 เคส + LPG ppm + Temp (ไม่มี ADC)
void drawOLED_byCase(bool smokeFlag, bool gasFlag, int lpg_ppm, float tempC)
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // หัวเรื่องตามเคส
  display.setTextSize(2);
  display.setCursor(0, 0);
  if (smokeFlag && gasFlag)
    display.print("SMOKE+GAS");
  else if (smokeFlag)
    display.print("SMOKE");
  else if (gasFlag)
    display.print("GAS LEAK");
  else
    display.print("SAFE");

  // มุมบนขวา: อุณหภูมิ
  display.setTextSize(1);
  display.setCursor(90, 0);
  display.print("T:");
  if (isnan(tempC))
    display.print("--");
  else
  {
    display.print((int)roundf(tempC));
    display.print("C");
  }

  // กลางจอ: LPG ppm
  if (lpg_ppm < 0)
    lpg_ppm = 0;
  if (lpg_ppm > 9999)
    lpg_ppm = 9999;
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("LPG (MQ6): ");
  display.setTextSize(2);
  display.setCursor(0, 42);
  display.print(lpg_ppm);
  display.print("ppm");

  display.display();
}

/* -------- Setup / Loop -------- */
void setup()
{
  Serial.begin(9600);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("OLED init failed");
  }
  else
  {
    display.clearDisplay();
    display.display();
  }

  analogSetPinAttenuation(MQ2_AO_PIN, ADC_11db);
  analogSetPinAttenuation(MQ6_AO_PIN, ADC_11db);

  pinMode(MQ2_DO_PIN, INPUT);
  pinMode(MQ6_DO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(MQ2_DO_PIN), mq2_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MQ6_DO_PIN), mq6_isr, CHANGE);

  ledcAttachPin(BUZZER_PIN, BUZZ_CH);
  dht.begin();

  // ฟิตกราฟสำหรับ MQ-6 LPG จากสองจุด (ปรับค่าให้ตรงกับกราฟของคุณ)
  curve_mq6_LPG = mq_fit_curve(LPG_PPM_1, LPG_RATIO_1, LPG_PPM_2, LPG_RATIO_2);

  tStageStart = millis();
  stage = WARMUP;
  Serial.println("Start warm-up...");
}

void loop()
{
  unsigned long now = millis();

  // -------- Read MQ2/MQ6 (ADC) --------
  int rawMQ2 = analogRead(MQ2_AO_PIN);
  int rawMQ6 = analogRead(MQ6_AO_PIN);

  if (emaMQ2 < 0)
    emaMQ2 = rawMQ2;
  else
    emaMQ2 += EMA_ALPHA_MQ2 * (rawMQ2 - emaMQ2);
  if (emaMQ6 < 0)
    emaMQ6 = rawMQ6;
  else
    emaMQ6 += EMA_ALPHA_MQ6 * (rawMQ6 - emaMQ6);

  // -------- Read DHT11 (interval) --------
  if (now >= nextDhtReadAt)
  {
    float t = dht.readTemperature(); // °C
    if (!isnan(t))
      dhtTempC = t;
    nextDhtReadAt = now + DHT_READ_INTERVAL_MS;
  }

  // -------- Update buzzer pattern --------
  buzUpdate(now);

  switch (stage)
  {
  case WARMUP:
  {
    drawOLED_byCase(false, false, 0, dhtTempC);
    if (now - tStageStart >= WARMUP_MS)
    {
      stage = CALIBRATE;
      tStageStart = now;
      calibSumMQ2 = 0;
      calibN_MQ2 = 0;
      calibSumMQ6 = 0;
      calibN_MQ6 = 0;
      Serial.println("Warm-up done. Calibrating...");
    }
  }
  break;

  case CALIBRATE:
  {
    // เก็บค่า baseline
    calibSumMQ2 += rawMQ2;
    ++calibN_MQ2;
    calibSumMQ6 += rawMQ6;
    ++calibN_MQ6;

    // แสดงหน้าจอง่าย ๆ ระหว่างคาลิเบรต
    drawOLED_byCase(false, false, 0, dhtTempC);

    if (now - tStageStart >= CALIB_MS)
    {
      baselineMQ2 = (calibN_MQ2 > 0) ? (float)calibSumMQ2 / calibN_MQ2 : 50.0f;
      baselineMQ6 = (calibN_MQ6 > 0) ? (float)calibSumMQ6 / calibN_MQ6 : 50.0f;
      if (baselineMQ2 < 50)
        baselineMQ2 = 50;
      if (baselineMQ6 < 50)
        baselineMQ6 = 50;

      // === คำนวณ R0 สำหรับ MQ-6 จาก baseline (อากาศสะอาด) ===
      float vout6_base = (baselineMQ6 / 4095.0f) * VREF_ADC;
      R0_mq6 = mq_compute_Rs(vout6_base, VC_SENSOR, RL_OHM);
      if (R0_mq6 < 1.0f)
        R0_mq6 = 1.0f; // กันศูนย์/ค่าเพี้ยน

      stage = RUN;
      tStageStart = now;
      Serial.printf("Baseline MQ2=%.1f MQ6=%.1f  |  R0_MQ6=%.1f ohm\n", baselineMQ2, baselineMQ6, R0_mq6);
    }
  }
  break;

  case RUN:
  {
    float ratioMQ2 = (baselineMQ2 > 0) ? emaMQ2 / baselineMQ2 : 0.0f;
    float ratioMQ6 = (baselineMQ6 > 0) ? emaMQ6 / baselineMQ6 : 0.0f;

    // -------- DO (Interrupt + Debounce) --------
    int do2_raw = digitalRead(MQ2_DO_PIN);
    int do6_raw = digitalRead(MQ6_DO_PIN);
    bool doMQ2 = DO_ACTIVE_HIGH ? (do2_raw == HIGH) : (do2_raw == LOW);
    bool doMQ6 = DO_ACTIVE_HIGH ? (do6_raw == HIGH) : (do6_raw == LOW);

    static unsigned long do2_hold_until = 0, do6_hold_until = 0;
    if (mq2_irq)
    {
      mq2_irq = false;
      if (now - mq2_irq_time <= DO_DEBOUNCE_MS)
        do2_hold_until = now + DO_DEBOUNCE_MS;
    }
    if (mq6_irq)
    {
      mq6_irq = false;
      if (now - mq6_irq_time <= DO_DEBOUNCE_MS)
        do6_hold_until = now + DO_DEBOUNCE_MS;
    }
    if (now < do2_hold_until)
      doMQ2 = true;
    if (now < do6_hold_until)
      doMQ6 = true;

    // -------- Analog rules --------
    bool analogOnMQ2 = ((emaMQ2 > baselineMQ2 + ABS_ON_MQ2) && (ratioMQ2 >= RATIO_ON_MQ2));
    bool analogOffMQ2 = (ratioMQ2 <= RATIO_OFF_MQ2);

    bool analogOnMQ6 = ((emaMQ6 > baselineMQ6 + ABS_ON_MQ6) && (ratioMQ6 >= RATIO_ON_MQ6));
    bool analogOffMQ6 = (ratioMQ6 <= RATIO_OFF_MQ6);

    // -------- State machines --------
    if (!smoke)
    {
      if (doMQ2 || analogOnMQ2)
      {
        smoke = true;
        Serial.println("ALARM: SMOKE");
      }
    }
    else
    {
      if (!doMQ2 && analogOffMQ2)
      {
        smoke = false;
        Serial.println("CLEAR: SMOKE");
      }
    }

    if (!gasLeak)
    {
      if (doMQ6 || analogOnMQ6)
      {
        gasLeak = true;
        Serial.println("ALARM: GAS");
      }
    }
    else
    {
      if (!doMQ6 && analogOffMQ6)
      {
        gasLeak = false;
        Serial.println("CLEAR: GAS");
      }
    }

    // -------- Baseline drift (ช้า ๆ) --------
    if (!smoke)
      baselineMQ2 += 0.0005f * (emaMQ2 - baselineMQ2);
    if (!gasLeak)
      baselineMQ6 += 0.0005f * (emaMQ6 - baselineMQ6);

    // -------- คำนวณ LPG ppm จาก MQ-6 --------
    float vout6 = (emaMQ6 / 4095.0f) * VREF_ADC; // โวลต์ที่ AO
    float Rs6 = mq_compute_Rs(vout6, VC_SENSOR, RL_OHM);
    float ratio6 = (R0_mq6 > 0.0f) ? (Rs6 / R0_mq6) : 0.0f;
    float ppm_LPG = (ratio6 > 0.0f) ? mq_ppm_from_ratio(ratio6, curve_mq6_LPG) : 0.0f;
    if (ppm_LPG < 0.0f)
      ppm_LPG = 0.0f;
    if (ppm_LPG > 20000.0f)
      ppm_LPG = 20000.0f; // ครอบบนกันเลขล้น

    // -------- Buzzer --------
    if (smoke && gasLeak)
    {
      int f = (levelToFreqFromBaseline(emaMQ2, baselineMQ2) + levelToFreqFromBaseline(emaMQ6, baselineMQ6)) / 2;
      buzStart(f, 120, 40);
    }
    else if (smoke)
    {
      int f = levelToFreqFromBaseline(emaMQ2, baselineMQ2);
      buzStart(f, 120, 40);
    }
    else if (gasLeak)
    {
      int f = levelToFreqFromBaseline(emaMQ6, baselineMQ6);
      buzStart(f, 200, 80);
    }
    else
    {
      buzStop();
    }

    // -------- OLED: แสดง ppm --------
    int ppm_disp = (int)roundf(ppm_LPG);
    drawOLED_byCase(smoke, gasLeak, ppm_disp, dhtTempC);
  }
  break;
  }

  delay(20);
}