// Teensy 4.1 + H-bridge (type TB6612) - Canal A
// PWMA = 23 (A9), INA1 = 21 (A7), INA2 = 22 (A8)

const uint8_t PIN_PWMA = 22;  // PWM capable
const uint8_t PIN_INA1 = 20;  // direction
const uint8_t PIN_INA2 = 21;  // direction

const uint8_t PIN_MODE_DOWN = A16;  // from GPIO25
const uint8_t PIN_MODE_UP   = A17;  // from GPIO24

// Réglages utilisateur
const int SPEED = 100;        // 0..255 (≈ vitesse). Monte si besoin.
const uint32_t RUN_MS = 5000; // durée avance/retour en millisecondes

int old_state = LOW;
unsigned long stateChangeMillis = 0;

const uint32_t CMD_STABLE_MS = 80;

int lastUp = 0, lastDown = 0;
uint32_t lastChangeMs = 0;

void setup() {
  pinMode(PIN_INA1, OUTPUT); digitalWrite(PIN_INA1, LOW);
  pinMode(PIN_INA2, OUTPUT); digitalWrite(PIN_INA2, LOW);
  pinMode(PIN_PWMA, OUTPUT); analogWrite(PIN_PWMA, 0);

  pinMode(PIN_MODE_DOWN, INPUT_PULLDOWN);
  pinMode(PIN_MODE_UP,   INPUT_PULLDOWN);
  lastUp = digitalRead(PIN_MODE_UP);
  lastDown = digitalRead(PIN_MODE_DOWN);
  lastChangeMs = millis();

  analogWriteFrequency(PIN_PWMA, 20000);
  
  stopCoast(); // moteur au repos au démarrage
}

// Conduite signée : >0 avance, <0 recule, 0 = coast
void drive(int speedSigned) {
  if (speedSigned > 0) {
    digitalWrite(PIN_INA1, HIGH);
    digitalWrite(PIN_INA2, LOW);
    analogWrite(PIN_PWMA, constrain(speedSigned, 0, 255));
  } else if (speedSigned < 0) {
    digitalWrite(PIN_INA1, LOW);
    digitalWrite(PIN_INA2, HIGH);
    analogWrite(PIN_PWMA, constrain(-speedSigned, 0, 255));
  } else {
    stopCoast();
  }
}

// Met le pont en "coast" (00) et coupe le PWM
void stopCoast() {
  digitalWrite(PIN_INA1, LOW);
  digitalWrite(PIN_INA2, LOW);
  analogWrite(PIN_PWMA, 0);
}

void loop() {
  int up = digitalRead(PIN_MODE_UP);
  int down = digitalRead(PIN_MODE_DOWN);

  if (up != lastUp || down != lastDown) {
    lastUp = up;
    lastDown = down;
    lastChangeMs = millis();
  }

  // If inputs haven't been stable long enough, do nothing (stop)
  if (millis() - lastChangeMs < CMD_STABLE_MS) {
    drive(0);
    delay(5);
    return;
  }

  // Use lastUp/lastDown as the "accepted" stable command
  if (lastDown == HIGH && lastUp == LOW) {
    drive(-SPEED);
  } else if (lastUp == HIGH && lastDown == LOW) {
    drive(SPEED);
  } else {
    drive(0);
  }

  delay(5);

}



