// MOTOR A (KIRI)
#define AIN1 26
#define AIN2 25
#define PWMA 14

// MOTOR B (KANAN)
#define BIN1 33
#define BIN2 32
#define PWMB 27

// STANDBY
#define STBY 13

// PWM
#define PWM_FREQ 20000
#define PWM_RES 8
#define SPEED 180

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("=== TB6612 2 MOTOR TEST ===");

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.println("TB6612 ENABLED");

  // PWM attach (API BARU)
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  Serial.println("PWM A & B READY");
}

void loop() {
  // MAJU
  Serial.println("MAJU");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, SPEED);
  ledcWrite(PWMB, SPEED);
  delay(3000);

  // STOP
  Serial.println("STOP");
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  delay(1000);

  // MUNDUR
  Serial.println("MUNDUR");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMA, SPEED);
  ledcWrite(PWMB, SPEED);
  delay(3000);

  // STOP
  Serial.println("STOP");
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  delay(2000);

  // BELOK KANAN
  Serial.println("BELOK KANAN");
  ledcWrite(PWMA, SPEED);
  ledcWrite(PWMB, 80);
  delay(2000);

  // BELOK KIRI
  Serial.println("BELOK KIRI");
  ledcWrite(PWMA, 80);
  ledcWrite(PWMB, SPEED);
  delay(2000);

  // STOP
  Serial.println("STOP");
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  delay(3000);
}
