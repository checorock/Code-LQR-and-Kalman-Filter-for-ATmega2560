#include <Wire.h>
#include <VL53L0X.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
VL53L0X sensor;

#define ECD_A 2
#define ECD_B 3
#define D1 6
#define D2 7
#define PWM 13

float hys = 10.3;
const float Ts = 0.01;
const int Tm = 10;
volatile long CNT = 0;

float Kf[4] = {-1.3547,  -10.5998,  -34.6591,   -8.8763};
float Ref = 0.30;
float u = 0;

BLA::Matrix<4,4> Ad = {
  1,    0.0093,   -0.0001,   -0.0000,
  0,    0.8688,   -0.0119,   -0.0001,
  0,    0.0010,    1.0008,    0.0100,
  0,    0.2007,    0.1683,    1.0008};

BLA::Matrix<4,1> Bd = {
  0.0001,
  0.0262,
  -0.0002,
  -0.0401};

BLA::Matrix<2,4> C = {
  1,0,0,0,
  0,0,1,0};

BLA::Matrix<4,4> Q = {
  1,   0,    0,    0,
  0,    1e-2, 0,    0,
  0,    0,    1,  0,
  0,    0,    0,    0.01};

BLA::Matrix<2,2> R = {
  0.0001,0,
  0,0.0001};

BLA::Matrix<4,4> I = {
  1,0,0,0,
  0,1,0,0,
  0,0,1,0,
  0,0,0,1};

BLA::Matrix<4,1> x = {0.22,0,0,0};
BLA::Matrix<4,4> P = I;
BLA::Matrix<4,4> AP, At;
BLA::Matrix<4,2> K;
BLA::Matrix<2,1> y, yhat;
BLA::Matrix<2,2> S;


unsigned long tAnterior = 0;
const int OrdenFiltro = 7;
double yk = 0.0;
double xf[7] = {0};
const double b[7] = {
0.142386908288827163,
0.142856919905681196,
0.143139372760745009,
0.143233598089493264,
0.143139372760745009,
0.142856919905681196,
0.142386908288827163
};

void Kalman(float yk, float ang_rad, float u) {
  y(0,0) = yk;
  y(1,0) = ang_rad;
  x = Ad * x + Bd * u;
  AP = Ad * P;
  At = ~Ad;
  P = AP * At + Q;
  yhat = C * x;
  S = C * P * ~C + R;
  K = P * ~C * Inverse(S);
  x = x + K * (y - yhat);
  P = (I - K * C) * P;
}

void StateFeedback() {
  float e = Ref-x(0);
  u = Kf[0]*e+ Kf[1]*x(1) + Kf[2]*x(2) + Kf[3]*x(3);
}

void ControlOutput(float u) {
  if (u > 0.01) {
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
  } else if (u < -0.01) {
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
  } else {
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
  }
  float pwm_val = fabs(u) + hys;
  if (pwm_val > 50.0) pwm_val = 50.0;
  if (pwm_val < hys + 0.01) pwm_val = 0;
  int pwm_out = pwm_val;
  analogWrite(PWM, pwm_out);
}

void A_isr() {
  if (digitalRead(ECD_A) == digitalRead(ECD_B)) CNT++;
  else CNT--;}
void B_isr() {
  if (digitalRead(ECD_A) != digitalRead(ECD_B)) CNT++;
  else CNT--;}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);

  sensor.startContinuous();
  pinMode(ECD_A, INPUT);
  pinMode(ECD_B, INPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(PWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECD_A), A_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECD_B), B_isr, CHANGE);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  analogWrite(PWM, 0);
}

void FiltroDist(float dist_m) {
    xf[0] = dist_m;
    yk = 0.0;
    for (int k = 0; k < OrdenFiltro; k++) yk += b[k] * xf[k];
    for (int k = OrdenFiltro - 1; k > 0; k--) xf[k] = xf[k - 1];}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();
  uint16_t dist_mm = sensor.readRangeContinuousMillimeters();
  float dist_m = (dist_mm / 1000.0);
if (now - last >= Tm) {
  last = now;
  FiltroDist(dist_m);
  float ang_rad = CNT * (2.0 * PI / (256.0 * 4.0));
  float ang_deg = ang_rad * 180.0 / PI;
  Kalman(yk, ang_rad, u);
  StateFeedback();
  ControlOutput(u);
  Serial.print(x(0),4); Serial.print("\t");
  Serial.print(x(1),4); Serial.print("\t");
  Serial.print(x(2),4); Serial.print("\t");
  Serial.print(x(3),4); Serial.print("\t");
  Serial.println(u,4);
}
}