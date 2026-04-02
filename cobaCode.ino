#include <Arduino.h>
#include <SparkFun_TB6612.h>

/************ DEBUG ************/
#define DEBUG 1   // 1 = aktif, 0 = mati

/************ MOTOR ************/
#define AIN1 21
#define BIN1 18
#define AIN2 22
#define BIN2 5
#define PWMA 23
#define PWMB 15
#define STBY 19

Motor motorRight = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motorLeft  = Motor(BIN1, BIN2, PWMB, 1, STBY);

/************ SENSOR ************/
const uint8_t SensorCount = 8;
const uint8_t sensorPins[SensorCount] = {34,35,32,33,25,26,27,14};

uint16_t sensorValues[SensorCount];
uint16_t minValues[SensorCount];
uint16_t maxValues[SensorCount];

uint16_t threshold = 500; // 🔥 TUNING: ubah sesuai lighting

/************ PID ************/
// 🔥 START dari ini dulu
float Kp = 25;   // respon utama (belok cepat atau lambat)
float Ki = 0;    // biasanya 0 dulu (jarang dipakai di line follower)
float Kd = 15;   // stabilisasi (ngurangin goyang)

float error = 0, lastError = 0;
float integral = 0;

/************ SPEED ************/
int baseSpeed = 120;   // 🔥 TUNING: makin tinggi makin cepat tapi rawan keluar
int maxSpeed  = 200;

/************ STATE ************/
enum RobotState {
  FOLLOW_LINE,
  INTERSECTION,
  SHARP_TURN,
  LOST_LINE
};

RobotState state = FOLLOW_LINE;

/************ STRATEGI ************/
bool PRIORITY_LEFT = true; // true = belok kiri dulu

/************ DEBUG VAR ************/
int g_active, g_left, g_right;

/************ READ SENSOR ************/
void readSensors()
{
  for(int i=0;i<SensorCount;i++)
  {
    int raw = analogRead(sensorPins[i]);

    raw = constrain(raw, minValues[i], maxValues[i]);
    int mapped = map(raw, minValues[i], maxValues[i], 0, 1000);

    sensorValues[i] = 1000 - mapped; // hitam = tinggi
  }
}

/************ HITUNG ERROR ************/
float calculateError()
{
  long numerator = 0;
  long denominator = 0;

  for(int i=0;i<SensorCount;i++)
  {
    numerator += (i * 1000) * sensorValues[i];
    denominator += sensorValues[i];
  }

  if(denominator == 0) return lastError;

  float position = numerator / (float)denominator;

  return position - 3500; // tengah sensor
}

/************ DETEKSI KONDISI ************/
void detectState()
{
  int active = 0;
  int leftSide = 0;
  int rightSide = 0;

  for(int i=0;i<SensorCount;i++)
  {
    if(sensorValues[i] > threshold)
    {
      active++;
      if(i < 4) leftSide++;
      else rightSide++;
    }
  }

  g_active = active;
  g_left = leftSide;
  g_right = rightSide;

  if(active == 0)
    state = LOST_LINE;

  else if(active >= 6)
  {
    if(abs(leftSide - rightSide) <= 2)
      state = INTERSECTION;
    else
      state = SHARP_TURN;
  }
  else
    state = FOLLOW_LINE;
}

/************ DEBUG PRINT ************/
void debugPrint(int leftSpeed, int rightSpeed)
{
#if DEBUG
  Serial.print("STATE: ");

  switch(state)
  {
    case FOLLOW_LINE: Serial.print("FOLLOW"); break;
    case INTERSECTION: Serial.print("INTERSECTION"); break;
    case SHARP_TURN: Serial.print("SHARP"); break;
    case LOST_LINE: Serial.print("LOST"); break;
  }

  Serial.print(" | Err: "); Serial.print(error);
  Serial.print(" | Act: "); Serial.print(g_active);
  Serial.print(" | L: "); Serial.print(g_left);
  Serial.print(" | R: "); Serial.print(g_right);

  Serial.print(" | ML: "); Serial.print(leftSpeed);
  Serial.print(" | MR: "); Serial.print(rightSpeed);

  Serial.println();
#endif
}

/************ KALIBRASI ************/
void calibrate()
{
  for(int i=0;i<SensorCount;i++)
  {
    minValues[i] = 4095;
    maxValues[i] = 0;
  }

  unsigned long start = millis();

  while(millis() - start < 5000)
  {
    for(int i=0;i<SensorCount;i++)
    {
      int v = analogRead(sensorPins[i]);
      minValues[i] = min(minValues[i], (uint16_t)v);
      maxValues[i] = max(maxValues[i], (uint16_t)v);
    }
  }
}

/************ SETUP ************/
void setup()
{
  Serial.begin(115200);

  for(int i=0;i<SensorCount;i++)
    pinMode(sensorPins[i], INPUT);

  Serial.println("Kalibrasi sensor...");
  calibrate();
  Serial.println("Selesai");
}

/************ LOOP ************/
void loop()
{
  readSensors();
  detectState();

  error = calculateError();

  /******** LOST LINE ********/
  if(state == LOST_LINE)
  {
    Serial.println("!!! LOST LINE !!!");

    if(lastError > 0)
    {
      motorLeft.drive(100);
      motorRight.drive(-100);
    }
    else
    {
      motorLeft.drive(-100);
      motorRight.drive(100);
    }
    return;
  }

  /******** INTERSECTION ********/
  if(state == INTERSECTION)
  {
    Serial.println(">>> INTERSECTION <<<");

    if(PRIORITY_LEFT)
    {
      motorLeft.drive(80);
      motorRight.drive(150);
    }
    else
    {
      motorLeft.drive(150);
      motorRight.drive(80);
    }

    delay(50); // 🔥 TUNING: 30–100 ms
    return;
  }

  /******** SHARP TURN ********/
  if(state == SHARP_TURN)
  {
    Serial.println("### SHARP TURN ###");
    baseSpeed = 90; // pelan biar nggak kebuang
  }
  else
  {
    baseSpeed = 120;
  }

  /******** PID ********/
  float P = error;

  integral += error;  // ⚠️ biasanya ga dipakai (biar aman)
  float D = error - lastError;

  float correction = (Kp * P) + (Ki * integral) + (Kd * D);

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);

  debugPrint(leftSpeed, rightSpeed);

  lastError = error;

  delay(10); // biar serial kebaca
}
