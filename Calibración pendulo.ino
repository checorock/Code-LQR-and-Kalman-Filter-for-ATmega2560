#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
const byte ECD_A = 2, ECD_B = 3;
const unsigned long Tm = 10;
const int CPR = 256 * 4;
volatile long CNT = 0;
unsigned long lastTime = 0;

float yk = 0.0, yk_1 = 0.0;
float alfa = 0.1;
float uk = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();  
  sensor.init();
  delay(100);
  sensor.setTimeout(500);
  sensor.startContinuous();
  pinMode(ECD_A, INPUT);
  pinMode(ECD_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECD_A), A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECD_B), B, CHANGE);
  uk = 0;
  yk = 0;
  yk_1 = yk;
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= Tm) {
    lastTime = now;

    // Distancia en mm
    uint16_t dist_mm = sensor.readRangeContinuousMillimeters();
    // Distancia en metros
    float dist_m = dist_mm / 1000.0f;

    // Ángulo
    float ang_rad = CNT * (2.0f * PI / CPR);
    float ang_deg = ang_rad * 180.0f / PI;
     uk = dist_m;
     yk = alfa * uk + ((1 - alfa) * yk_1);
 
  yk_1 = yk;
 
  Serial.print(yk);
 Serial.print('\t');
    Serial.print(dist_m, 4);    Serial.print('\t');
    Serial.println(ang_deg, 4);
    }
}

void A() {
  if (digitalRead(ECD_A) == digitalRead(ECD_B))
    CNT++;
  else
    CNT--;}
void B() {
  if (digitalRead(ECD_A) != digitalRead(ECD_B))
    CNT++;
  else
    CNT--;}