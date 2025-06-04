int count1 = 0;
  int count2 = 0;
  int count3 = 0;
  int count4 = 0;
  int count5 = 0;
  int count6 = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(22,INPUT);
  pinMode(24,INPUT);
  pinMode(26,INPUT);
  pinMode(28,INPUT);
  pinMode(30,INPUT);
  pinMode(32,INPUT);
}

void loop() {
  if (digitalRead(22) == HIGH & count1 ==0){
    count1 = 1;
    Serial.println("Sensor 1 received obstacle!");
  }
  if (digitalRead(24) == HIGH & count2 ==0){
    count2 = 1;
    Serial.println("Sensor 2 received obstacle!");
  }
  if (digitalRead(26) == HIGH & count3 ==0){
    count3 = 1;
    Serial.println("Sensor 3 received obstacle!");
  }
  if (digitalRead(28) == HIGH & count4 ==0){
    count4 = 1;
    Serial.println("Sensor 4 received obstacle!");
  }
  if (digitalRead(30) == HIGH & count5 ==0){
    count5 = 1;
    Serial.println("Sensor 5 received obstacle!");
  }
  if (digitalRead(32) == HIGH & count6 ==0){
    count6 = 1;
    Serial.println("Sensor 6 received obstacle!");
  }

}
