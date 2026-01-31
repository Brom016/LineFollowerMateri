// PIN configuration
#define AIN1 26
#define AIN2 25
#define PWMA 14
#define STBY 27

// PWM
#define PWM_CHANNEL 0
#define PWM_FREQ 20000
#define PWM_RES 8  // 0-255

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // Aktifkan TB6612

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CHANNEL);
}

void loop() {
  // MOTOR MAJU
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CHANNEL, 180); // Kecepatan
  delay(3000);

  // STOP
  ledcWrite(PWM_CHANNEL, 0);
  delay(1000);

  // MOTOR MUNDUR
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWM_CHANNEL, 180);
  delay(3000);

  // STOP
  ledcWrite(PWM_CHANNEL, 0);
  delay(2000);
}
