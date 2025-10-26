// Motor Pins
int ena_front = 5;
int in1_front = 6;
int in2_front = 7;

void setup() {
  pinMode(ena_front, OUTPUT);
  pinMode(in1_front, OUTPUT);
  pinMode(in2_front, OUTPUT);

}

void loop() {
  digitalWrite(in1_front, HIGH);
  digitalWrite(in2_front, LOW);
  analogWrite(ena_front, 255);
  delay(2000);
}