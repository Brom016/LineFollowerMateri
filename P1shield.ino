#define M1_PWM 11
#define M1_DIR 12

#define M2_PWM 3
#define M2_DIR 13

void setup() {
  Serial.begin(9600);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);

  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  Serial.println("Test Motor M1 dan M2 dimulai...");
}

void loop() {

  Serial.println("Motor MAJU");

  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);

  analogWrite(M1_PWM, 200);
  analogWrite(M2_PWM, 200);

  delay(3000);

  Serial.println("Motor STOP");

  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);

  delay(2000);

  Serial.println("Motor MUNDUR");

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  analogWrite(M1_PWM, 200);
  analogWrite(M2_PWM, 200);

  delay(3000);
}