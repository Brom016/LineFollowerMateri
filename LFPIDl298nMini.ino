#include <Arduino.h>
#include <TaskScheduler.h>
#include <QuickPID.h>

/************ MOTOR CLASS (PWM di IN) ************/
class Motor {
  int in1, in2, channel;

public:
  Motor(int pin1, int pin2, int ch) {
    in1 = pin1;
    in2 = pin2;
    channel = ch;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    ledcSetup(channel, 20000, 8);
    ledcAttachPin(in1, channel); // PWM di IN1
  }

  void drive(int speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
      digitalWrite(in2, LOW);
      ledcWrite(channel, speed);
    }
    else if (speed < 0) {
      digitalWrite(in2, HIGH);
      ledcWrite(channel, -speed);
    }
    else {
      digitalWrite(in2, LOW);
      ledcWrite(channel, 0);
    }
  }
};

/************ PIN MOTOR ************/
#define IN1 21  // kanan PWM
#define IN2 22

#define IN3 18  // kiri PWM
#define IN4 5

Motor motorRight(IN1, IN2, 0);
Motor motorLeft (IN3, IN4, 1);

/************ SENSOR ************/
const uint8_t SensorCount = 8;
const uint8_t sensorPins[SensorCount] = {34,35,32,33,25,26,27,14};

uint16_t sensorValues[SensorCount];
uint16_t minValues[SensorCount];
uint16_t maxValues[SensorCount];

float sensorWeights[SensorCount] = {12.5,2.5,0.5,0.1,0.1,0.5,2.5,12.5};

/************ SYSTEM ************/
Scheduler runner;

int32_t lineError = 0;
uint16_t lineThreshold = 800;
uint8_t noLineCount = 0;

/************ PID ************/
float Kp = 0.8;
float Ki = 0.0;
float Kd = 0.15;

float input = 0;
float output = 0;
float setpoint = 0;

QuickPID pid(&input, &output, &setpoint, Kp, Ki, Kd, QuickPID::Action::direct);

/************ SPEED ************/
int baseSpeed = 140;

/************ READ SENSOR ************/
void readSensors() {
  for(int i=0;i<SensorCount;i++) {
    int raw = analogRead(sensorPins[i]);
    raw = constrain(raw, minValues[i], maxValues[i]);
    int mapped = map(raw, minValues[i], maxValues[i], 0, 1000);
    sensorValues[i] = 1000 - mapped;
  }
}

/************ ERROR ************/
void readAndCalculateError() {
  readSensors();

  bool lineExists = false;
  lineError = 0;

  for(int i=0;i<SensorCount;i++) {
    lineError += (i - 3.5) * sensorValues[i] * sensorWeights[i];

    if(sensorValues[i] > lineThreshold)
      lineExists = true;
  }

  if(!lineExists) {
    noLineCount++;
    if(noLineCount > 20)
      lineError = (lineError > 0) ? 50000 : -50000;
  } else {
    noLineCount = 0;
  }
}

/************ PID COMPUTE ************/
void computePID() {
  input = lineError;
  pid.Compute();
}

/************ MOTOR CONTROL ************/
void driveMotor() {
  int leftSpeed  = baseSpeed + output;
  int rightSpeed = baseSpeed - output;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motorLeft.drive(leftSpeed);
  motorRight.drive(rightSpeed);
}

/************ TASK ************/
Task t1(10, TASK_FOREVER, &readAndCalculateError, &runner, true);
Task t2(10, TASK_FOREVER, &computePID, &runner, true);
Task t3(10, TASK_FOREVER, &driveMotor, &runner, true);

/************ CALIBRATION ************/
void calibrateSensors() {
  for(int i=0;i<SensorCount;i++) {
    minValues[i]=4095;
    maxValues[i]=0;
  }

  unsigned long start = millis();
  while(millis()-start < 5000) {
    for(int i=0;i<SensorCount;i++) {
      int v = analogRead(sensorPins[i]);
      minValues[i] = min(minValues[i],v);
      maxValues[i] = max(maxValues[i],v);
    }
  }
}

/************ SETUP ************/
void setup() {
  Serial.begin(115200);

  for(int i=0;i<SensorCount;i++)
    pinMode(sensorPins[i], INPUT);

  Serial.println("Calibrating...");
  calibrateSensors();
  Serial.println("Done");

  pid.SetMode(QuickPID::Control::automatic);
}

/************ LOOP ************/
void loop() {
  runner.execute();
}
