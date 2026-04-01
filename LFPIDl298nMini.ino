// ================================================================
//  LINE FOLLOWER — ESP32 DevKit V1 + L298N Mini + TCRT5000 x8
//  Compatible: ESP32 Arduino Core v3.x (Arduino IDE 1.8.x / 2.x)
// ================================================================

#include <Arduino.h>
#include <QuickPID.h>

// ----------------------------------------------------------------
//  KONFIGURASI PIN MOTOR (L298N Mini)
// ----------------------------------------------------------------
#define PWM_R   21
#define DIR_R   22
#define PWM_L   18
#define DIR_L    5

#define PWM_FREQ 20000
#define PWM_RES  8      // 8-bit → duty 0–255

// ----------------------------------------------------------------
//  KONFIGURASI SENSOR TCRT5000 (8 channel, kiri → kanan)
// ----------------------------------------------------------------
const uint8_t SENSOR_COUNT = 8;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {34, 35, 32, 33, 25, 26, 27, 14};

const int16_t SENSOR_POS[SENSOR_COUNT] = {
  -3500, -2500, -1500, -500,
    500,  1500,  2500,  3500
};

uint16_t sensorVal[SENSOR_COUNT];
uint16_t calMin[SENSOR_COUNT];
uint16_t calMax[SENSOR_COUNT];

const uint16_t LINE_THRESHOLD = 400;

// ----------------------------------------------------------------
//  KONFIGURASI PID
//  FIX: Kp dinaikkan agar koreksi belok lebih kuat
// ----------------------------------------------------------------
float Kp = 0.18f;   // ← naik dari 0.06 (terlalu lemah)
float Ki = 0.00f;
float Kd = 1.20f;   // ← naik proporsional dengan Kp

float pidInput    = 0.0f;
float pidOutput   = 0.0f;
float pidSetpoint = 0.0f;

QuickPID pid(&pidInput, &pidOutput, &pidSetpoint,
             Kp, Ki, Kd, QuickPID::Action::direct);

// ----------------------------------------------------------------
//  KONFIGURASI KECEPATAN
//  FIX: BASE_SPEED diturunkan agar ada "ruang" koreksi yang cukup
// ----------------------------------------------------------------
const int BASE_SPEED = 100;   // ← turun dari 130 (terlalu cepat untuk belok)
const int MAX_SPEED  = 220;   // ← dibatasi agar tidak slip

// ----------------------------------------------------------------
//  STATE TRACKING
// ----------------------------------------------------------------
float   lastError = 0.0f;
int8_t  lastDir   = 0;
uint8_t noLineCnt = 0;
const uint8_t NO_LINE_LIMIT = 20;

// ================================================================
//  MOTOR CLASS — ESP32 Core v3 API
// ================================================================
class Motor {
  uint8_t _pwm, _dir;
public:
  Motor(uint8_t pwm, uint8_t dir) : _pwm(pwm), _dir(dir) {}

  void begin() {
    pinMode(_dir, OUTPUT);
    ledcAttach(_pwm, PWM_FREQ, PWM_RES);
    stop();
  }

  void drive(int speed) {
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    if (speed >= 0) {
      digitalWrite(_dir, LOW);
      ledcWrite(_pwm, (uint8_t)speed);
    } else {
      digitalWrite(_dir, HIGH);
      ledcWrite(_pwm, (uint8_t)(-speed));
    }
  }

  void stop() {
    digitalWrite(_dir, LOW);
    ledcWrite(_pwm, 0);
  }
};

Motor motorRight(PWM_R, DIR_R);
Motor motorLeft (PWM_L, DIR_L);

// ================================================================
//  BACA SENSOR
//  FIX: variabel 'mapped' dideklarasikan eksplisit
//       agar inversion mudah diaktifkan jika perlu
// ================================================================
void readSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t raw    = analogRead(SENSOR_PINS[i]);
    raw             = constrain(raw, calMin[i], calMax[i]);
    uint16_t mapped = map(raw, calMin[i], calMax[i], 0, 1000);

    // TCRT5000 normal: garis hitam → ADC tinggi → sensorVal tinggi ✓
    sensorVal[i] = mapped;

    // Jika robot belok ke arah SALAH → aktifkan baris ini:
    // sensorVal[i] = 1000 - mapped;
  }
}

// ================================================================
//  HITUNG ERROR — WEIGHTED AVERAGE POSITION
//  error = Σ(posisi[i] × nilai[i]) / Σ(nilai[i])
// ================================================================
float calculateError() {
  readSensors();

  int32_t  weightedSum = 0;
  uint32_t totalWeight = 0;
  bool     lineFound   = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t v = sensorVal[i];
    weightedSum += (int32_t)SENSOR_POS[i] * v;
    totalWeight += v;
    if (v > LINE_THRESHOLD) lineFound = true;
  }

  if (!lineFound) {
    noLineCnt++;
    if (noLineCnt > NO_LINE_LIMIT) {
      return (lastDir >= 0) ? 4000.0f : -4000.0f;
    }
    return lastError;
  }

  noLineCnt = 0;

  float err = (totalWeight > 0)
              ? (float)weightedSum / (float)totalWeight
              : 0.0f;

  if      (err >  400) lastDir =  1;
  else if (err < -400) lastDir = -1;
  else                 lastDir =  0;

  lastError = err;
  return err;
}

// ================================================================
//  KALIBRASI
//  Gerakkan robot zig-zag di atas garis selama 5 detik
//  Semua sensor HARUS membaca warna putih DAN hitam minimal 1x
// ================================================================
void calibrateSensors(uint16_t durationMs = 5000) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    calMin[i] = 4095;
    calMax[i] = 0;
  }

  Serial.println("=== KALIBRASI ===");
  Serial.println("Gerakkan robot zig-zag di atas garis...");

  unsigned long t       = millis();
  unsigned long lastSec = 0;

  while (millis() - t < durationMs) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      uint16_t v = analogRead(SENSOR_PINS[i]);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
    unsigned long elapsed = millis() - t;
    if (elapsed / 1000 != lastSec) {
      lastSec = elapsed / 1000;
      Serial.printf("  %lu detik...\n", lastSec);
    }
    delay(2);
  }

  Serial.println("\nHasil kalibrasi:");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t range = calMax[i] - calMin[i];
    Serial.printf("  S%d: min=%-4d  max=%-4d  rentang=%-4d  %s\n",
                  i, calMin[i], calMax[i], range,
                  (range < 300) ? "<-- WARNING: rentang kecil!" : "OK");
    if (range < 100) {
      calMin[i] = 0;
      calMax[i] = 4095;
    }
  }
  Serial.println("=================\n");
}

// ================================================================
//  DEBUG SERIAL — 10 Hz
//  FIX: tampilkan nilai sensorVal mentah untuk diagnosa
// ================================================================
void debugPrint(float err, int lSpeed, int rSpeed) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  // Visualisasi bar sensor
  Serial.print("[");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    Serial.print(sensorVal[i] > LINE_THRESHOLD ? "X" : ".");
  Serial.print("]  ");

  // Error, PID output, kecepatan motor
  Serial.printf("Err:%7.0f  PID:%7.1f  L:%4d  R:%4d  diff:%4d  ",
                err, pidOutput, lSpeed, rSpeed, abs(lSpeed - rSpeed));

  if      (noLineCnt > NO_LINE_LIMIT) Serial.print("[ LOST  ]");
  else if (err >  400)                Serial.print("[ KANAN ]");
  else if (err < -400)                Serial.print("[ KIRI  ]");
  else                                Serial.print("[ LURUS ]");

  Serial.println();
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== LINE FOLLOWER START ===");

  motorRight.begin();
  motorLeft.begin();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    pinMode(SENSOR_PINS[i], INPUT);

  calibrateSensors(5000);

  // FIX: limit PID output = MAX_SPEED - BASE_SPEED
  int pidLimit = MAX_SPEED - BASE_SPEED;  // = 120
  pid.SetOutputLimits(-pidLimit, pidLimit);
  pid.SetSampleTimeUs(10000);             // 10 ms = 100 Hz
  pid.SetMode(QuickPID::Control::automatic);

  // Verifikasi PID limit di serial
  Serial.printf("PID limit: +/-%d\n", pidLimit);
  Serial.printf("Kecepatan: base=%d, max=%d\n", BASE_SPEED, MAX_SPEED);
  Serial.printf("Kp=%.3f  Ki=%.3f  Kd=%.3f\n\n", Kp, Ki, Kd);

  // Test motor
  Serial.println("Test motor L...");
  motorLeft.drive(120); delay(500); motorLeft.stop(); delay(300);

  Serial.println("Test motor R...");
  motorRight.drive(120); delay(500); motorRight.stop(); delay(300);

  Serial.println("Test maju...");
  motorLeft.drive(120); motorRight.drive(120);
  delay(800);
  motorLeft.stop(); motorRight.stop();
  delay(500);

  Serial.println("Mulai!\n");
}

// ================================================================
//  LOOP — 100 Hz
// ================================================================
void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();
  if (now - lastLoop < 10) return;
  lastLoop = now;

  // 1. Baca sensor & hitung error posisi
  float err = calculateError();

  // 2. PID compute
  pidInput = err;
  pid.Compute();

  // 3. Hitung kecepatan motor
  int leftSpeed  = BASE_SPEED + (int)pidOutput;
  int rightSpeed = BASE_SPEED - (int)pidOutput;
  leftSpeed  = constrain(leftSpeed,  0, MAX_SPEED);  // tidak mundur saat lurus
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);

  // 4. Debug
  debugPrint(err, leftSpeed, rightSpeed);
}

// ================================================================
//  DIAGNOSA — Baca output Serial Monitor saat robot di tikungan:
//
//  NORMAL (PID bekerja):
//    [....XXXX]  Err:  1800  PID:  32.4  L: 132  R:  68  diff:  64  [ KANAN ]
//    → diff > 0, L > R saat belok kanan ✓
//
//  MASALAH 1 — error tidak berubah (sensor tidak baca garis):
//    [........]  Err:     0  PID:   0.0  L: 100  R: 100  diff:   0
//    → Solusi: ulang kalibrasi, pastikan robot zig-zag di atas garis
//
//  MASALAH 2 — error ada tapi diff kecil (Kp terlalu kecil):
//    [....XXXX]  Err:  1800  PID:   5.4  L: 105  R:  95  diff:  10
//    → Solusi: naikkan Kp (0.18 → 0.25 → 0.35)
//
//  MASALAH 3 — arah belok terbalik:
//    [XXXX....]  (garis di kiri) tapi robot belok kanan
//    → Solusi: aktifkan baris sensorVal[i] = 1000 - mapped;
//
// ----------------------------------------------------------------
//  TUNING PID:
//  1. Kp=0.10, Ki=0, Kd=0  → naikkan Kp sampai bergoyang, turun 20%
//  2. Naikkan Kd sampai halus
//  3. Ki hampir tidak pernah perlu untuk line follower
// ================================================================
