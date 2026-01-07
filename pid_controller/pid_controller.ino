/*

4 wheel drive PID control

  MOTOR LAYOUT:
  1 0-------0 2
       pi
     battery
  3 0-------0 4

*/

// pin config
#define MOTOR1INA 22
#define MOTOR1INB 24
#define MOTOR1EN 6
#define MOTOR1ENCA 18
#define MOTOR1ENCB 43

#define MOTOR2INA 26
#define MOTOR2INB 28
#define MOTOR2EN 5
#define MOTOR2ENCA 21
#define MOTOR2ENCB 51

#define MOTOR3INA 30
#define MOTOR3INB 32
#define MOTOR3EN 7
#define MOTOR3ENCA 3
#define MOTOR3ENCB 42

#define MOTOR4INA 34
#define MOTOR4INB 36
#define MOTOR4EN 8
#define MOTOR4ENCA 2
#define MOTOR4ENCB 48

int pos = 0;
long prev_t = 0;
float e_prev = 0;
float e_integral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR4INA, OUTPUT);
  pinMode(MOTOR4INB, OUTPUT);
  pinMode(MOTOR4EN, OUTPUT);
  pinMode(MOTOR4ENCA,INPUT);
  pinMode(MOTOR4ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR4ENCA), readEncoder, RISING);
}

void loop() {

  // PID constants
  float kp = 6;
  float kd = 1;
  float ki = 0.05;

  // delta t
  long curr_t = micros();
  float dt = ((float)(curr_t-prev_t))/1.0e6;
  prev_t = curr_t;

  // target

  // float t = curr_t * 1e-6;
  // float freq = 0.25;
  // float omega = 2 * PI * freq;
  // int target = (int)(600 * cos(omega * t));
  int target = 1200;


  // error
  int e = target - pos;

  // derivative
  float dedt = (e-e_prev)/(dt);

  // integral
  e_integral = e_integral + e*dt;

  // control signal
  float u = kp*e + kd*dedt + ki*e_integral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, MOTOR4EN, MOTOR4INA, MOTOR4INB);

  e_prev = e;

  Serial.print(0);
  Serial.print(" ");
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.println(1200);

}

void setMotor(int dir, int pwm_val, int en, int ina, int inb ){
  analogWrite(en, pwm_val);
  if (dir == 1) {
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);
  } else if (dir == -1) {
    digitalWrite(ina, LOW);
    digitalWrite(inb, HIGH);
  } else {
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(MOTOR4ENCB);
  if (b>0) {
    pos++;
  } else {
    pos--;
  }
}