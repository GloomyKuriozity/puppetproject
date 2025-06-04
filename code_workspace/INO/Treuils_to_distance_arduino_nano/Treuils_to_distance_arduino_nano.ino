// Motor 1 pins
const int motor1PWM = 3;
const int motor1Control1 = 2;
const int motor1Control2 = 4;

// Motor 2 pins
const int motor2PWM = 5;
const int motor2Control1 = 7;
const int motor2Control2 = 8;

// Motor 3 pins
const int motor3PWM = 6;
const int motor3Control1 = 10;
const int motor3Control2 = 11;

// Motor 4 pins
const int motor4PWM = 9;
const int motor4Control1 = 12;
const int motor4Control2 = 13;

int speed = 70;

// Pin definitions for the IR sensors
const int sensorPins[7] = {A0, A1, A2, A3, A4, A5, A6};  // Analog pins for 7 sensors
float distances[7];  // To store the calculated distances in cm

// Set distance thresholds (can be adjusted based on sensor range)
float minDist = 8;      // Minimum distance (in cm) for max forward speed
float maxDist = 12;     // Maximum distance (in cm) for max reverse speed

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
    case 3:
      digitalWrite(motor3PWM, speed);
      digitalWrite(motor3Control1, direction);
      digitalWrite(motor3Control2, !direction);
      break;
    case 4:
      analogWrite(motor4PWM, speed);
      digitalWrite(motor4Control1, direction);
      digitalWrite(motor4Control2, !direction);
      break;
    case 5:
      analogWrite(motor1PWM, speed);
      digitalWrite(motor1Control1, direction);
      digitalWrite(motor1Control2, !direction);
      analogWrite(motor2PWM, speed);
      digitalWrite(motor2Control1, direction);
      digitalWrite(motor2Control2, !direction);
      analogWrite(motor3PWM, speed);
      digitalWrite(motor3Control1, direction);
      digitalWrite(motor3Control2, !direction);
      analogWrite(motor4PWM, speed);
      digitalWrite(motor4Control1, direction);
      digitalWrite(motor4Control2, !direction);
      break;
    default:
      break;
  }
}

void read_ir_sensors_value() {
  // Read the values from each sensor
  for (int i = 0; i < 7; i++) {
    int sensorValue = analogRead(sensorPins[i]);  // Read analog value (0 to 1023)
    float voltage = sensorValue * (5.0 / 1023.0);  // Convert to voltage (assuming 5V system)

    // Convert voltage to distance
    float distance = convertToDistance(voltage);

    distances[i] = distance;  // Store distance
  }
}

// Function to convert the analog value to distance (in cm)
float convertToDistance(float voltage) {
  // Based on the image provided, the curve is approximately inverse proportional,
  // fitting an exponential decay model for the distance to voltage curve.
  
  if (voltage > 3.5) voltage = 3.5;  // Limit to the max voltage in the graph

  // Approximation formula (this could be adjusted to fit the exact curve more precisely)
  float distance = 27.86 * pow(voltage, -1.15);  // Example values derived from the curve

  return distance; // return distance in cm
}

// Function to map sensor distance to motor speed with linear interpolation and a dead zone
int mapDistanceToSpeed(float distance, float minDist, float maxDist, float deadZoneMin, float deadZoneMax, int maxSpeed) {
  // If within the dead zone, return 0 speed (stop)
  if (distance >= deadZoneMin && distance <= deadZoneMax) {
    return 0;
  }

  // Clamp distance to the expected range
  if (distance > maxDist) distance = maxDist;
  if (distance < minDist) distance = minDist;

  // Map the distance to speed (linear interpolation)
  // When distance is at maxDist, motor speed is -maxSpeed (reverse)
  // When distance is at minDist, motor speed is +maxSpeed (forward)
  int speed = map(distance, minDist, maxDist, -maxSpeed, maxSpeed);
  
  return speed;
}

void setup() {  
  // Set PWM pins as output
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor3PWM, OUTPUT);
  pinMode(motor4PWM, OUTPUT);

  // Set control pins as output
  pinMode(motor1Control1, OUTPUT);
  pinMode(motor1Control2, OUTPUT);
  pinMode(motor2Control1, OUTPUT);
  pinMode(motor2Control2, OUTPUT);
  pinMode(motor3Control1, OUTPUT);
  pinMode(motor3Control2, OUTPUT);
  pinMode(motor4Control1, OUTPUT);
  pinMode(motor4Control2, OUTPUT);

  // Initialize motor control pins to LOW
  digitalWrite(motor1Control1, LOW);
  digitalWrite(motor1Control2, LOW);
  digitalWrite(motor2Control1, LOW);
  digitalWrite(motor2Control2, LOW);
  digitalWrite(motor3Control1, LOW);
  digitalWrite(motor3Control2, LOW);
  digitalWrite(motor4Control1, LOW);
  digitalWrite(motor4Control2, LOW);
}

int controlMotor(float side1, float side2){
  // Check if either sensor is detecting a close object
  if (side1 < minDist || side2 < minDist){
    return speed;  // Move forward
  }
  // Check if either sensor is detecting a distant object
  else if (side1 > maxDist || side2 > maxDist){
    return -speed;  // Reverse
  }
  else {
    return 0;  // Stay still
  }
}



void loop() {
  read_ir_sensors_value();

  // Calculate motor speeds based on sensor distances
  int speedMotor1 = controlMotor((distances[4] + distances[5]) / 2, distances[6]);
  int speedMotor2 = controlMotor((distances[0] + distances[1]) / 2, distances[6]);
  int speedMotor3 = controlMotor((distances[4] + distances[5]) / 2, (distances[2] + distances[3]) / 2);
  int speedMotor4 = controlMotor((distances[2] + distances[3]) / 2, (distances[0] + distances[1]) / 2);

  // Set motor speeds based on the calculated interpolated values
  setMotor(1, speedMotor1);
  setMotor(2, speedMotor2);
  setMotor(3, speedMotor3);
  setMotor(4, speedMotor4);

  delay(100);  // Small delay to ensure proper motor timing
}
