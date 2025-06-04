// Motor 1 pins
const int motor1PWM = 3;
const int motor1Control1 = 4;
const int motor1Control2 = 5;

// Motor 2 pins
const int motor2PWM = 6;
const int motor2Control1 = 7;
const int motor2Control2 = 8;

int speed = 100;

void setup() {
  Serial.begin(9600);
  
  // Set PWM pins as output
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  // Set control pins as output
  pinMode(motor1Control1, OUTPUT);
  pinMode(motor1Control2, OUTPUT);
  pinMode(motor2Control1, OUTPUT);
  pinMode(motor2Control2, OUTPUT);

  // Initialize motor control pins to LOW
  digitalWrite(motor1Control1, LOW);
  digitalWrite(motor1Control2, LOW);
  digitalWrite(motor2Control1, LOW);
  digitalWrite(motor2Control2, LOW);
}

// Function to set motor speed and direction
void setMotor(int motorNum, int speed) {
  bool direction;
  int controlPin1;
  int controlPin2;
  int pwmPin;

  if (speed < 0) {
    direction = false;
    speed = -speed;
  } else {
    direction = true;
    speed = speed;
  }

  switch (motorNum) {
    case 1:
      analogWrite(motor1PWM, speed);
      digitalWrite(motor1Control1, direction);
      digitalWrite(motor1Control2, !direction);
      break;
    case 2:
      analogWrite(motor2PWM, speed);
      digitalWrite(motor2Control1, direction);
      digitalWrite(motor2Control2, !direction);
      break;
    case 5:
      analogWrite(motor1PWM, speed);
      digitalWrite(motor1Control1, direction);
      digitalWrite(motor1Control2, !direction);
      analogWrite(motor2PWM, speed);
      digitalWrite(motor2Control1, direction);
      digitalWrite(motor2Control2, !direction);
      break;
    default:
      break;
  }
}

void loop() {
  setMotor(1,speed);
  delay(100);
}
