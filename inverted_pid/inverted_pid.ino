#include <Wire.h>

// === Konfigurasi Stepper Motor ===
#define directionPin 2
#define stepPin 3
#define limitSwitchPin 7   // limit titik nol di kanan
#define stepsPerRevolution 6400
#define distancePerRevolution 37.7  // mm per putaran

// === Parameter PID ===
float Kp = 25.0;
float Ki = 2.0;
float Kd = 1.5;

// === Waktu ===
float Ts = 0.005;
unsigned long Ts_us = 5000;

// === Variabel State Pendulum ===
float theta = 0;
float theta_prev = 0;
float theta_dot = 0;
float e = 0, e_prev = 0, I = 0;
float theta_offset = 0;   // kalibrasi saat tegak

// === Posisi Cart ===
long stepCount = 0;
long centerStep = 0;    // titik tengah setelah homing
float x_mm = 0;
float v_mmps = 0;
float prev_x_mm = 0;

// === Step Service ===
unsigned long lastStep = 0;
float targetStepRate = 0;  // steps/s
float MAX_RATE = 15000;    // batas

// === Konversi ===
float stepsPerMM = stepsPerRevolution / distancePerRevolution;

// === AS5600 ===
#define AS5600_ADDR 0x36
#define ANGLE_H 0x0E

// === Filter derivatif sederhana ===
float alpha_d = 0.15;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limitSwitchPin, INPUT_PULLUP);

  homing();
  prev_x_mm = 0;
}

void loop() {
  static unsigned long last = micros();
  unsigned long now = micros();
  if (now - last >= Ts_us) {
    last += Ts_us;

    updateState();
    pidControl();
  }

  stepService();
}

// === Update sensor & state ===
void updateState() {
  theta = readAS5600Radians() - theta_offset;
  theta = wrapPi(theta);

  float raw_dtheta = (theta - theta_prev) / Ts;
  theta_dot = (1.0 - alpha_d) * raw_dtheta + alpha_d * theta_dot;
  theta_prev = theta;

  x_mm = (stepCount - centerStep) / stepsPerMM;
  v_mmps = (x_mm - prev_x_mm) / Ts;
  prev_x_mm = x_mm;
}

// === PID ===
void pidControl() {
  e = -theta;
  I += e * Ts;
  float D = (e - e_prev) / Ts;
  e_prev = e;

  float u = Kp*e + Ki*I + Kd*D;
  targetStepRate = u * 1000.0;

  if (targetStepRate >  MAX_RATE) targetStepRate =  MAX_RATE;
  if (targetStepRate < -MAX_RATE) targetStepRate = -MAX_RATE;

  if (digitalRead(limitSwitchPin) == LOW && targetStepRate > 0) {
    targetStepRate = 0;
  }
}

// === Step Non-Blocking ===
void stepService() {
  if (targetStepRate == 0) return;

  digitalWrite(directionPin, targetStepRate > 0 ? HIGH : LOW);
  unsigned long period = (unsigned long)(1000000.0 / abs(targetStepRate));
  unsigned long now = micros();
  if (now - lastStep >= period) {
    lastStep = now;
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    stepCount += (targetStepRate > 0 ? 1 : -1);
  }
}

// === Homing ===
void homing() {
  digitalWrite(directionPin, HIGH);
  while (digitalRead(limitSwitchPin) == HIGH) {
    pulseStep(4000);
  }
  stepCount = 0;
  delay(200);
  moveSteps(2000, LOW, 4000);
  centerStep = stepCount;
}

void pulseStep(float rate) {
  static unsigned long last = 0;
  unsigned long now = micros();
  unsigned long period = (unsigned long)(1000000.0 / rate);
  if (now - last >= period) {
    last = now;
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
  }
}

void moveSteps(long steps, int dir, float rate) {
  digitalWrite(directionPin, dir);
  unsigned long period = (unsigned long)(1000000.0 / rate);
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(period);
    stepCount += (dir == HIGH ? 1 : -1);
  }
}

// === AS5600 ===
float readAS5600Radians() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint16_t high = Wire.read();
  uint16_t low = Wire.read();
  uint16_t angle12 = ((high & 0x0F) << 8) | low;
  float angle = (float)angle12 * (2.0 * PI / 4096.0);
  return angle - PI;
}

float wrapPi(float a) {
  while (a > PI)  a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}
