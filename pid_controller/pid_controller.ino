/*

4 wheel drive PID control

  MOTOR LAYOUT:
  1 0-------0 2
       pi
     battery
  3 0-------0 4

*/

// pin config
// #define MOTOR1INA 22
// #define MOTOR1INB 24
// #define MOTOR1EN 6
// #define MOTOR1ENCA 18
// #define MOTOR1ENCB 43

// #define MOTOR2INA 26
// #define MOTOR2INB 28
// #define MOTOR2EN 5
// #define MOTOR2ENCA 21
// #define MOTOR2ENCB 51

// #define MOTOR3INA 30
// #define MOTOR3INB 32
// #define MOTOR3EN 7
// #define MOTOR3ENCA 3
// #define MOTOR3ENCB 42

// #define MOTOR4INA 34
// #define MOTOR4INB 36
// #define MOTOR4EN 8
// #define MOTOR4ENCA 2
// #define MOTOR4ENCB 48

#include <util/atomic.h>

#define NUMMOTORS 4
const int in1[] = {22, 26, 30, 34};
const int in2[] = {24, 28, 32, 36};
const int en[] = {6, 5, 7, 8};
const int enca[] = {18, 21, 3, 2};
const int encb[] = {43, 51, 42, 48};

// globals
long prev_t = 0;
volatile int pos[] = {0, 0, 0, 0};

// targets
float target_f[] = {0.0, 0.0, 0.0, 0.0};
long target[] = {0, 0, 0, 0};

void setTarget(float t, float dt) {
  float position_change[4] = {0.0, 0.0, 0.0, 0.0};
  float pulses_per_turn = 508;
  float pulses_per_meter = pulses_per_turn * 4.75089382365;

  t = fmod(t, 12);
  float velocity = 0.25;

  if (t < 4) {
     
  } else if (t < 5.2) {
    for (int i = 0; i < 4; i++) {
      position_change[i] = velocity * dt * pulses_per_meter;
    }
  } else if (t < 6.4) {
    for (int i = 0; i < 4; i++) {
      position_change[i] = -velocity * dt * pulses_per_meter;
    }
  }

  for (int i = 0; i < 4; i++) {
    target_f[i] = target_f[i] + position_change[i];
  }
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
  target[2] = (long) target_f[2];
  target[3] = (long) target_f[3];
}

class Controller {
  private:
    float kp, kd, ki, umax;
    float e_prev, e_integral;
  public:
    Controller() : kp(1), kd(0), ki(0), umax(255), e_prev(0.0), e_integral(0.0){}

  void setParams(float kp_new, float kd_new, float ki_new, float umax_new) {
    kp = kp_new;
    kd = kd_new;
    ki = ki_new;
    umax = umax_new;
  }

  void getSignal(int value, int target, float dt, int &pwr, int &dir) {
    // error
    int e = target - value;

      // derivative
    float dedt = (e-e_prev)/(dt);

    // integral
    e_integral = e_integral + e*dt;

    // control signal
    float u = kp*e + kd*dedt + ki*e_integral;

    // motor power
    pwr = (int) fabs(u);
    if (pwr > umax) {
      pwr = umax;
    }

    // motor direction
    dir = 1;
    if (u < 0) {
      dir = -1;
    }

    e_prev = e;
  }
};

Controller controller[NUMMOTORS];

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUMMOTORS; i++) {
    pinMode(in1[i], OUTPUT);
    pinMode(in2[i], OUTPUT);
    pinMode(en[i], OUTPUT);
    pinMode(enca[i],INPUT);
    pinMode(encb[i],INPUT);
  }

  controller[0].setParams(3, 0.5, 0.03, 255);
  controller[1].setParams(3, 0.5, 0.03, 255);
  controller[2].setParams(3, 0.5, 0.03, 255);
  controller[3].setParams(3, 0.5, 0.03, 255);

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
}

void loop() {

  long curr_t = micros();
  float dt = ((float) (curr_t - prev_t))/(1.0e6);
  prev_t = curr_t;

    // targets
  setTarget(curr_t/1.0e6, dt);

  int posi[NUMMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int i = 0; i < NUMMOTORS; i++) {
      posi[i] = pos[i];
    }
  }

  for (int i = 0; i < NUMMOTORS; i++) {
    int pwr, dir;
    controller[i].getSignal(posi[i], target[i], dt, pwr, dir);
    setMotor(dir, pwr, en[i], in1[i], in2[i]);
  }

  for (int i = 0; i < NUMMOTORS; i++) {
    Serial.print(target[i]);
    Serial.print(" ");
    Serial.print(posi[i]);
    Serial.print(" ");
  }
  Serial.println();

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

template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b>0) {
    pos[j]++;
  } else {
    pos[j]--;
  }
}