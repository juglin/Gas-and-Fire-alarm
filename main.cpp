// ESP32 + MQ-6 (Analog) -> estimate ppm (LPG/propane/butane)
// Amber for Chip (Sakkarin)
// NOTE: You MUST scale AOUT (0-5V) down to <=3.3V before connecting to ESP32 ADC.

#include <Preferences.h>
Preferences prefs;

// ==== Pin config (edit to match your wiring) ====
const int MQ6_AO_PIN = 34;     // ADC pin (use 34/35/32/33/36/39 for input-only)
const int MQ6_DO_PIN = 13;     // optional: digital threshold from module's LM393

// ==== Electrical & ADC config ====
const float VCC_SENSOR = 5.0;       // supply to the sensor module (usually 5V)
const float ADC_VREF   = 3.3;       // ESP32 ADC reference (approx)
const int   ADC_MAX    = 4095;      // 12-bit
const float RL_OHMS    = 10000.0;   // load resistor on module (typ. 5k~10k) <-- adjust if known
const float DIVIDER_K  = 3.3/5.0;   // voltage divider scale: V_adc = Vout_sensor * DIVIDER_K
// If you used a different resistor divider, set the exact scale here.
// Example: if AOUT 0–5V is scaled to 0–3.0V, use DIVIDER_K = 3.0/5.0.

// ==== Sampling & filtering ====
const int   N_SAMPLES      = 50;     // number of ADC samples per reading
const int   PRINT_INTERVAL = 1000;   // ms

// ==== Calibration (R0) storage ====
const char* NVS_NAMESPACE = "mq6";
const char* NVS_KEY_R0    = "R0";
float R0 = NAN;                      // sensor baseline (to be determined)

// Option A: quick "clean-air" calibration (approx)
// For MQ-6, Rs/R0 in clean air is ~10 (rough ballpark; adjust if you have the exact spec)
const float CLEAN_AIR_FACTOR = 10.0;

// ==== LPG curve (Rs/R0 vs ppm) on log-log line ====
// We'll compute slope m and intercept b from two reference points
// Defaults roughly from typical MQ-6 curves (adjust to your calibration):
//   at 1000 ppm -> Rs/R0 ~ 1.0
//   at 10000 ppm -> Rs/R0 ~ 0.36
const float P1_PPM   = 1000.0;
const float R1_RATIO = 1.0;
const float P2_PPM   = 10000.0;
const float R2_RATIO = 0.36;

float CURVE_m = 0.0f, CURVE_b = 0.0f;

// ==== Helpers ====
float readADC_AvgVoltage() {
  // read N samples and average (in volts at ESP32 ADC pin)
  long acc = 0;
  for (int i = 0; i < N_SAMPLES; ++i) {
    acc += analogRead(MQ6_AO_PIN);
    delayMicroseconds(500);
  }
  float raw = (float)acc / N_SAMPLES;
  float v_adc = raw * (ADC_VREF / ADC_MAX);       // Volts at ESP32 pin (<=3.3 V)
  // Convert back to sensor's AOUT (0-5V) using divider factor
  float v_out_sensor = v_adc / DIVIDER_K;         // Volts at module analog out
  // Clamp to [0, VCC_SENSOR]
  if (v_out_sensor < 0) v_out_sensor = 0;
  if (v_out_sensor > VCC_SENSOR) v_out_sensor = VCC_SENSOR;
  return v_out_sensor;
}

float calcRs(float v_out_sensor) {
  // Voltage divider: Vout = Vcc * (RL / (RS + RL))  =>  RS = RL*(Vcc/Vout - 1)
  // Handle Vout == 0 to avoid div by zero
  if (v_out_sensor <= 0.001) return 1e9; // ~infinite when saturated low
  float Rs = RL_OHMS * (VCC_SENSOR / v_out_sensor - 1.0f);
  if (Rs < 1.0f) Rs = 1.0f; // safety clamp
  return Rs;
}

void computeCurve() {
  // Fit log10(Rs/R0) = m*log10(ppm) + b
  float x1 = log10(P1_PPM);
  float y1 = log10(R1_RATIO);
  float x2 = log10(P2_PPM);
  float y2 = log10(R2_RATIO);
  CURVE_m = (y2 - y1) / (x2 - x1);
  CURVE_b = y1 - CURVE_m * x1;
}

float rsr0ToPPM(float rs_over_r0) {
  // ppm = 10^((log10(rs/r0) - b)/m)
  if (rs_over_r0 <= 0) rs_over_r0 = 1e-6;
  float log_ppm = (log10(rs_over_r0) - CURVE_b) / CURVE_m;
  float ppm = pow(10.0f, log_ppm);
  if (!isfinite(ppm)) ppm = 0;
  return ppm;
}

float loadR0FromNVS() {
  float r0 = NAN;
  if (prefs.begin(NVS_NAMESPACE, true)) { // read-only
    r0 = prefs.getFloat(NVS_KEY_R0, NAN);
    prefs.end();
  }
  return r0;
}

void saveR0ToNVS(float r0) {
  if (isnan(r0) || r0 <= 0) return;
  if (prefs.begin(NVS_NAMESPACE, false)) { // read-write
    prefs.putFloat(NVS_KEY_R0, r0);
    prefs.end();
  }
}

// Quick clean-air calibration: assumes Rs/R0 ≈ CLEAN_AIR_FACTOR in clean air
float calibrateR0_CleanAir(uint32_t ms = 5000) {
  // Warm-up a few seconds for stability (sensor should have been powered for minutes)
  uint32_t t0 = millis();
  long n = 0;
  double sumRs = 0.0;
  while (millis() - t0 < ms) {
    float vout = readADC_AvgVoltage();
    float rs   = calcRs(vout);
    sumRs += rs;
    n++;
    delay(50);
  }
  float Rs_air = (n > 0) ? (float)(sumRs / n) : NAN;
  if (!isfinite(Rs_air) || Rs_air <= 0) return NAN;
  return Rs_air / CLEAN_AIR_FACTOR;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC setup
  analogReadResolution(12);
  // If available in your core: set max range ~3.3V on this pin
  #ifdef ADC_11db
    analogSetPinAttenuation(MQ6_AO_PIN, ADC_11db);
  #endif

  pinMode(MQ6_DO_PIN, INPUT); // optional (digital threshold)

  computeCurve();

  // Try to load saved R0
  R0 = loadR0FromNVS();
  if (isnan(R0)) {
    Serial.println(F("R0 not found -> quick clean-air calibration (5s). Keep sensor in clean air..."));
    delay(1000);
    R0 = calibrateR0_CleanAir(5000);
    if (isfinite(R0) && R0 > 0) {
      saveR0ToNVS(R0);
      Serial.print(F("Calibrated R0 ≈ ")); Serial.print(R0, 1); Serial.println(F(" ohms (saved)"));
    } else {
      Serial.println(F("Calibration failed. Will still print raw values."));
    }
  } else {
    Serial.print(F("Loaded R0 from NVS: ")); Serial.print(R0, 1); Serial.println(F(" ohms"));
  }

  Serial.println(F("\nCommands (Serial):"));
  Serial.println(F("  c = recalibrate R0 in clean air (5s) and save"));
  Serial.println(F("  r = print current R0"));
  Serial.println(F("  ? = print help"));
  Serial.println();
}

uint32_t lastPrint = 0;

void loop() {
  // Handle simple serial commands
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c') {
      Serial.println(F("Recalibrating R0 in clean air (5s)..."));
      float newR0 = calibrateR0_CleanAir(5000);
      if (isfinite(newR0) && newR0 > 0) {
        R0 = newR0;
        saveR0ToNVS(R0);
        Serial.print(F("New R0 saved: ")); Serial.print(R0, 1); Serial.println(F(" ohms"));
      } else {
        Serial.println(F("Calibration failed."));
      }
    } else if (ch == 'r') {
      Serial.print(F("R0 = ")); Serial.print(R0, 1); Serial.println(F(" ohms"));
    } else if (ch == '?') {
      Serial.println(F("Commands: c(recal), r(print R0), ?(help)"));
    }
  }

  // Periodic reading
  uint32_t now = millis();
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;

    float vout  = readADC_AvgVoltage(); // sensor module AOUT (reconstructed, 0..5V)
    float rs    = calcRs(vout);
    int   dstate = digitalRead(MQ6_DO_PIN); // 0 = above threshold (gas detected), 1 = below

    Serial.print(F("AOUT=")); Serial.print(vout, 3); Serial.print(F(" V  "));

    Serial.print(F("Rs=")); 
    if (rs > 9.9e6) Serial.print(F(">9.9M"));
    else Serial.print(rs, 0);
    Serial.print(F("Ω  "));

    if (isfinite(R0) && R0 > 0) {
      float ratio = rs / R0;
      float ppm   = rsr0ToPPM(ratio);
      Serial.print(F("Rs/R0=")); Serial.print(ratio, 2); Serial.print(F("  "));

      // Clamp ppm to a readable range
      if (ppm < 0) ppm = 0;
      if (ppm > 20000) ppm = 20000;

      Serial.print(F("Est≈ ")); Serial.print(ppm, 0); Serial.print(F(" ppm"));
    } else {
      Serial.print(F("(R0 not set → showing raw only)"));
    }

    Serial.print(F("   DO=")); Serial.println(dstate == LOW ? F("TRIPPED") : F("OK"));
  }
}
