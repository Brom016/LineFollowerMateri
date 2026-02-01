// PIN configuration
#define AIN1 26
#define AIN2 25
#define PWMA 14
#define STBY 27

// PWM
#define PWM_FREQ 20000
#define PWM_RES 8  // duty 0–255
#define SPEED 180

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32 TB6612 Motor Test");

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.println("TB6612 ENABLED (STBY = HIGH)");

  // PWM API
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  Serial.println("PWM attached to pin 14");
}

void loop() {
  // MOTOR MAJU
  Serial.print("MOTOR MAJU | PWM = ");
  Serial.println(SPEED);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, SPEED);
  delay(3000);

  // STOP
  Serial.println("MOTOR STOP");
  ledcWrite(PWMA, 0);
  delay(1000);

  // MOTOR MUNDUR
  Serial.print("MOTOR MUNDUR | PWM = ");
  Serial.println(SPEED);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, SPEED);
  delay(3000);

  // STOP
  Serial.println("MOTOR STOP");
  ledcWrite(PWMA, 0);
  delay(2000);
}
