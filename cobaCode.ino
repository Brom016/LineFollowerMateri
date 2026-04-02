#include <Arduino.h>
#include <SparkFun_TB6612.h>

// ================================================================
//  DEBUG
// ================================================================
#define DEBUG            1        // 1 = aktif, 0 = mati
#define DEBUG_INTERVAL   100      // ms, max print 10x/detik

// ================================================================
//  PIN MOTOR
// ================================================================
#define AIN1  21
#define AIN2  22
#define PWMA  23
#define BIN1  18
#define BIN2  5
#define PWMB  15
#define STBY  19

Motor motorRight = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motorLeft  = Motor(BIN1, BIN2, PWMB, 1, STBY);

// ================================================================
//  PIN TOMBOL (BOOT button ESP32 = GPIO0, active LOW)
// ================================================================
#define BTN_PIN  0

// ================================================================
//  SENSOR
// ================================================================
const uint8_t  SENSOR_COUNT               = 8;
const uint8_t  sensorPins[SENSOR_COUNT]   = {34, 35, 32, 33, 25, 26, 27, 14};
const uint16_t THRESHOLD                  = 500;  // 0–1000, hitam = tinggi
const uint16_t MIN_CALIBRATION_RANGE      = 200;  // range minimum kalibrasi valid

uint16_t sensorValues[SENSOR_COUNT];
uint16_t minValues[SENSOR_COUNT];
uint16_t maxValues[SENSOR_COUNT];

// ================================================================
//  PID
// ================================================================
float Kp = 25.0f;
float Ki = 0.0f;
float Kd = 15.0f;

const float INTEGRAL_MAX = 500.0f;   // batas anti-windup

float error     = 0.0f;
float lastError = 0.0f;
float integral  = 0.0f;

// ================================================================
//  KECEPATAN
// ================================================================
const int BASE_SPEED_NORMAL = 120;  // kecepatan normal
const int BASE_SPEED_SHARP  = 90;   // melambat di tikungan tajam
const int MAX_SPEED         = 200;  // batas atas motor

// ================================================================
//  STATE MACHINE
// ================================================================
enum RobotState : uint8_t {
  FOLLOW_LINE,
  INTERSECTION,
  SHARP_TURN,
  LOST_LINE,
  STOPPED       // berhenti permanen setelah timeout LOST_LINE
};

RobotState state = FOLLOW_LINE;

// ================================================================
//  TIMING — semua non-blocking, tanpa delay()
// ================================================================
const uint32_t LOOP_INTERVAL_US   = 10000UL;  // 10ms = 100 Hz PID
const uint32_t INTERSECTION_MS    = 80UL;      // durasi aksi di persimpangan
const uint32_t LOST_TIMEOUT_MS    = 3000UL;   // berhenti jika garis hilang > 3 dtk

uint32_t lastLoopUs        = 0;
uint32_t intersectionStart = 0;  // 0 = belum masuk intersection ini
uint32_t lostStart         = 0;  // 0 = baru saja hilang
uint32_t lastDebugMs       = 0;

// ================================================================
//  STRATEGI PERSIMPANGAN
// ================================================================
bool PRIORITY_LEFT = true;   // true = kiri, false = kanan

// ================================================================
//  BACA & NORMALISASI SENSOR
// ================================================================
void readSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t raw = (uint16_t)analogRead(sensorPins[i]);
    raw = constrain(raw, minValues[i], maxValues[i]);

    // proteksi division-by-zero: terjadi bila kalibrasi kurang bergerak
    if (maxValues[i] == minValues[i]) {
      sensorValues[i] = 0;
      continue;
    }

    uint16_t mapped  = (uint16_t)map(raw, minValues[i], maxValues[i], 0, 1000);
    sensorValues[i]  = 1000 - mapped;   // inversi: hitam → nilai tinggi
  }
}

// ================================================================
//  HITUNG POSISI GARIS (weighted average)
// ================================================================
float calculateError() {
  long numerator   = 0;
  long denominator = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    numerator   += (long)(i * 1000) * sensorValues[i];
    denominator += sensorValues[i];
  }

  if (denominator == 0) return lastError;   // pertahankan error terakhir

  float position = (float)numerator / (float)denominator;
  return position - 3500.0f;               // titik tengah array = 0
}

// ================================================================
//  DETEKSI STATE
//  Menggunakan sensor edge (ujung) dan center untuk membedakan
//  intersection sejati dari tikungan tajam lebar.
// ================================================================
void detectState() {
  uint8_t active    = 0;
  uint8_t leftSide  = 0;
  uint8_t rightSide = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > THRESHOLD) {
      active++;
      if (i < 4) leftSide++;
      else       rightSide++;
    }
  }

  // sensor ujung dan tengah untuk membedakan intersection vs sharp turn
  bool leftEdge  = (sensorValues[0] > THRESHOLD || sensorValues[1] > THRESHOLD);
  bool rightEdge = (sensorValues[6] > THRESHOLD || sensorValues[7] > THRESHOLD);
  bool center    = (sensorValues[3] > THRESHOLD || sensorValues[4] > THRESHOLD);

  if (active == 0) {
    // tidak ada sensor aktif → garis hilang
    state = LOST_LINE;
  }
  else if (active >= 6) {
    // banyak sensor aktif: intersection = semua zona terkena (kiri + tengah + kanan)
    if (leftEdge && rightEdge && center)
      state = INTERSECTION;
    else
      state = SHARP_TURN;    // tikungan lebar ke satu sisi
  }
  else {
    // kondisi normal
    state             = FOLLOW_LINE;
    intersectionStart = 0;   // reset timer intersection
    lostStart         = 0;   // reset timer lost
  }
}

// ================================================================
//  HENTIKAN MOTOR
// ================================================================
void stopMotors() {
  motorLeft.drive(0);
  motorRight.drive(0);
}

// ================================================================
//  DEBUG PRINT (rate-limited, tidak spam Serial)
// ================================================================
void debugPrint(int leftSpeed, int rightSpeed) {
#if DEBUG
  uint32_t now = millis();
  if (now - lastDebugMs < DEBUG_INTERVAL) return;
  lastDebugMs = now;

  static const char* stateNames[] = {
    "FOLLOW", "INTERSECTION", "SHARP", "LOST", "STOPPED"
  };

  Serial.print("[");
  Serial.print(stateNames[(uint8_t)state]);
  Serial.print("] err:");
  Serial.print(error, 1);
  Serial.print("  P:");
  Serial.print(Kp * error, 1);
  Serial.print("  D:");
  Serial.print(Kd * (error - lastError), 1);
  Serial.print("  ML:");
  Serial.print(leftSpeed);
  Serial.print("  MR:");
  Serial.println(rightSpeed);
#else
  (void)leftSpeed;
  (void)rightSpeed;
#endif
}

// ================================================================
//  TUNGGU TOMBOL (BOOT button, active LOW)
// ================================================================
void waitForButton(const char* msg) {
  Serial.println(msg);
  pinMode(BTN_PIN, INPUT_PULLUP);
  while (digitalRead(BTN_PIN) == HIGH) {
    delay(10);
  }
  delay(300);   // debounce
}

// ================================================================
//  KALIBRASI
//  Robot berputar kiri-kanan di atas track selama 5 detik
//  agar setiap sensor menyentuh area hitam dan putih.
//  Setelah selesai, hasil divisualisasikan di Serial Monitor.
// ================================================================
void calibrate() {
  Serial.println("=== KALIBRASI DIMULAI ===");
  Serial.println("Robot berputar 5 detik. Pastikan sensor di atas track.");

  // reset nilai awal
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }

  uint32_t start   = millis();
  uint32_t phase   = 0;
  bool     goRight = true;

  while (millis() - start < 5000) {
    // alternasi arah tiap 400ms → sweep kiri–kanan
    uint32_t elapsed    = millis() - start;
    uint32_t thisPhase  = elapsed / 400;
    if (thisPhase != phase) {
      phase   = thisPhase;
      goRight = !goRight;
    }

    if (goRight) {
      motorLeft.drive(80);
      motorRight.drive(-80);
    } else {
      motorLeft.drive(-80);
      motorRight.drive(80);
    }

    // update min/max setiap iterasi
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      uint16_t v = (uint16_t)analogRead(sensorPins[i]);
      if (v < minValues[i]) minValues[i] = v;
      if (v > maxValues[i]) maxValues[i] = v;
    }
  }

  stopMotors();
  delay(300);

  // ---- validasi & laporan ----
  Serial.println("\nHasil Kalibrasi:");
  Serial.println("  Sensor  Min   Max  Range  Status");
  Serial.println("  ------  ---  ----  -----  ------");

  bool allOk = true;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t range = maxValues[i] - minValues[i];
    bool     ok    = (range >= MIN_CALIBRATION_RANGE);
    if (!ok) allOk = false;

    Serial.print("    S");
    Serial.print(i);
    Serial.print("    ");
    if (minValues[i] < 1000) Serial.print(" ");
    if (minValues[i] < 100)  Serial.print(" ");
    Serial.print(minValues[i]);
    Serial.print("  ");
    if (maxValues[i] < 1000) Serial.print(" ");
    Serial.print(maxValues[i]);
    Serial.print("  ");
    if (range < 100) Serial.print(" ");
    if (range < 10)  Serial.print(" ");
    Serial.print(range);
    Serial.print("   ");
    Serial.println(ok ? "OK" : "LEMAH ⚠");
  }

  Serial.println();
  if (!allOk) {
    Serial.println("PERINGATAN: Sensor dengan range kecil mungkin tidak terbaca.");
    Serial.println("Cek jarak sensor ke track dan ulangi kalibrasi jika perlu.");
  } else {
    Serial.println("Semua sensor terkalibrasi dengan baik.");
  }
  Serial.println("=========================");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== LINE FOLLOWER BOOT ===");

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // --- Kalibrasi ---
  waitForButton("Tekan BOOT untuk mulai kalibrasi...");
  calibrate();

  // --- Siap jalan ---
  waitForButton("Tekan BOOT untuk mulai jalan...");

  lastLoopUs = micros();
  Serial.println("GO!\n");
}

// ================================================================
//  LOOP UTAMA — 100 Hz (10ms interval via micros)
// ================================================================
void loop() {
  // --- rate limiter: jalankan hanya tiap LOOP_INTERVAL_US ---
  uint32_t now = micros();
  if (now - lastLoopUs < LOOP_INTERVAL_US) return;
  lastLoopUs = now;

  // --- baca sensor & deteksi state ---
  readSensors();
  detectState();
  error = calculateError();

  int leftSpeed  = 0;
  int rightSpeed = 0;

  // ============================================================
  //  STOPPED — berhenti permanen, butuh reset/power ulang
  // ============================================================
  if (state == STOPPED) {
    stopMotors();
    return;
  }

  // ============================================================
  //  LOST_LINE — putar mencari garis, berhenti jika timeout
  // ============================================================
  if (state == LOST_LINE) {
    if (lostStart == 0) lostStart = millis();

    if (millis() - lostStart > LOST_TIMEOUT_MS) {
      state = STOPPED;
      stopMotors();
      Serial.println("!!! STOPPED: garis hilang lebih dari 3 detik !!!");
      return;
    }

    // putar ke arah error terakhir agar balik ke garis
    if (lastError > 0) {
      motorLeft.drive(100);
      motorRight.drive(-100);
    } else {
      motorLeft.drive(-100);
      motorRight.drive(100);
    }

    debugPrint(0, 0);
    // lastError TIDAK diupdate saat LOST agar arah recovery konsisten
    return;
  }

  // ============================================================
  //  INTERSECTION — aksi selama INTERSECTION_MS, lalu lanjut
  // ============================================================
  if (state == INTERSECTION) {
    if (intersectionStart == 0) intersectionStart = millis();

    if (millis() - intersectionStart < INTERSECTION_MS) {
      if (PRIORITY_LEFT) {
        leftSpeed  = 80;
        rightSpeed = 150;
      } else {
        leftSpeed  = 150;
        rightSpeed = 80;
      }
      motorLeft.drive(leftSpeed);
      motorRight.drive(rightSpeed);
      debugPrint(leftSpeed, rightSpeed);
      return;
    }

    // durasi intersection selesai → reset timer, lanjut PID
    intersectionStart = 0;
  }

  // ============================================================
  //  FOLLOW_LINE & SHARP_TURN — kontrol PID
  // ============================================================
  int currentBase = (state == SHARP_TURN) ? BASE_SPEED_SHARP : BASE_SPEED_NORMAL;

  float P    = error;
  integral   = constrain(integral + error, -INTEGRAL_MAX, INTEGRAL_MAX);
  float D    = error - lastError;

  float correction = (Kp * P) + (Ki * integral) + (Kd * D);

  leftSpeed  = constrain(currentBase + (int)correction, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(currentBase - (int)correction, -MAX_SPEED, MAX_SPEED);

  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);

  debugPrint(leftSpeed, rightSpeed);

  lastError = error;
}
