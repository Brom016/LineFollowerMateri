void setMotorLeft(int speed) {          // B1
  speed = constrain(speed, -255, 255);   // B2
  if (speed > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    ledcWrite(ENA, speed);               // B3
  } else if (speed < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    ledcWrite(ENA, -speed);              // B4
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
  }
}

void setMotorRight(int speed) {         // B5
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    ledcWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    ledcWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    ledcWrite(ENB, 0);
  }
}

void stopAll() {
  setMotorLeft(0);
  setMotorRight(0);
}