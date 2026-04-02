#include <Arduino.h>
#include <SparkFun_TB6612.h>

/************ MOTOR PIN ************/
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

uint16_t threshold = 500;

/************ PID ************/
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, lastError = 0;
float integral = 0;

/************ SPEED ************/
int baseSpeed = 120;
int maxSpeed = 200;

/************ STATE ************/
enum RobotState {
  FOLLOW_LINE,
  INTERSECTION,
  LOST_LINE
};

RobotState state = FOLLOW_LINE;

/************ FUNCTION ************/
void readSensors()
{
  for(int i=0;i<SensorCount;i++)
  {
    int raw = analogRead(sensorPins[i]);
    raw = constrain(raw, minValues[i], maxValues[i]);

    int mapped = map(raw, minValues[i], maxValues[i], 0, 1000);
    sensorValues[i] = 1000 - mapped; // hitam tinggi
  }
}

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

  return position - 3500; // tengah = 3.5 * 1000
}

void detectState()
{
  int active = 0;

  for(int i=0;i<SensorCount;i++)
  {
    if(sensorValues[i] > threshold)
      active++;
  }

  if(active >= 6)
    state = INTERSECTION;
  else if(active == 0)
    state = LOST_LINE;
  else
    state = FOLLOW_LINE;
}

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

void setup()
{
  Serial.begin(115200);

  for(int i=0;i<SensorCount;i++)
    pinMode(sensorPins[i], INPUT);

  Serial.println("Kalibrasi...");
  calibrate();
  Serial.println("Selesai");
}

void loop()
{
  readSensors();
  detectState();

  if(state == INTERSECTION)
  {
    // STRATEGI: LURUS
    motorLeft.drive(baseSpeed);
    motorRight.drive(baseSpeed);
    return;
  }

  if(state == LOST_LINE)
  {
    // pakai last error buat cari lagi
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

  // FOLLOW LINE (PID)
  error = calculateError();

  float P = error;
  integral += error;
  float D = error - lastError;

  float correction = (Kp * P) + (Ki * integral) + (Kd * D);

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);

  lastError = error;
}
