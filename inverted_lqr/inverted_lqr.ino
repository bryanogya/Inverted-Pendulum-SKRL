// Arduino Uno + TB6600 + NEMA17 + AS5600 + Limit Switches
// LQR diskrit: u[k] = -K * x[k]
// x = [x_cart, v_cart, theta, theta_dot]

#include <Wire.h>

// -------------------- PIN DEFINISI --------------------
const int PIN_STEP = 2;     // ke TB6600 PUL+
const int PIN_DIR  = 3;     // ke TB6600 DIR+
const int PIN_ENA  = 4;     // optional ke TB6600 ENA+
const int PIN_LS_L = 8;     // limit kiri
const int PIN_LS_R = 9;     // limit kanan

// -------------------- AS5600 (I2C) --------------------
// Datasheet: alamat default 0x36, register ANGLE 0x0E-0x0F
const byte AS5600_ADDR = 0x36;
const byte ANGLE_H = 0x0E;

// -------------------- PARAMETER WAKTU --------------------
const float Ts = 0.005f;         // 5 ms
const unsigned long Ts_us = 5000; // microseconds

// -------------------- MEKANIK & SKALA --------------------
// Kalibrasi sesuai rail dan pulley/lead screw
const float STEPS_PER_METER = 80000.0f; // contoh GT2 2 mm, pulley 20T, microstep 16x: 80000 steps/m
const int   DIR_RIGHT = HIGH;
const int   DIR_LEFT  = LOW;

// Estimasi gaya->kecepatan stepper: u[N] -> v[m/s] -> step_rate[steps/s]
// Gunakan gain kasar lalu tuning
float U_TO_V = 0.2f;              // [m/s per N] perkiraan awal
float V_TO_STEPRATE = STEPS_PER_METER; // [steps/s per m/s]

// -------------------- LQR GAINS (ISI DARI PYTHON) ------
// u = -K*x, x=[x, v, theta, theta_dot]
float Kx = 0.0f;      // ganti sesuai hasil Python
float Kv = 0.0f;
float Kt = 0.0f;
float Ktd = 0.0f;

// -------------------- VARIABEL STATE --------------------
volatile long step_count = 0;   // posisi cart dari homing [steps]
float x_m = 0.0f;               // posisi [m]
float v_mps = 0.0f;             // kecepatan [m/s]

float theta_rad = 0.0f;         // sudut pendulum, 0 = tegak
float theta_dot = 0.0f;         // kecepatan sudut

// Filter derivatif sederhana
float prev_theta = 0.0f;
float alpha_d = 0.15f; // untuk low-pass pada theta_dot

// Step generation
unsigned long last_loop_us = 0;
unsigned long last_step_us = 0;
float target_step_rate = 0.0f; // steps/s
int dir_cmd = DIR_RIGHT;

// -------------------- LIMIT & HOMING --------------------
bool homed = false;
long x_center_steps = 0;  // tetapkan center setelah homing

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_ENA, LOW); // aktifkan (tergantung wiring)

  pinMode(PIN_LS_L, INPUT_PULLUP);
  pinMode(PIN_LS_R, INPUT_PULLUP);

  Wire.begin();
  Serial.begin(115200);

  delay(500);
  homing();
  last_loop_us = micros();
}

void loop() {
  unsigned long now = micros();
  if (now - last_loop_us >= Ts_us) {
    last_loop_us += Ts_us;

    // 1) Baca sensor
    float theta_new = readAS5600Radians();
    // Offset agar 0 = tegak, arah sesuai konvensi
    const float THETA_OFFSET = 0.0f; // kalibrasi saat tegak
    theta_rad = wrapPi(theta_new - THETA_OFFSET);

    // 2) Estimasi turunannya
    float raw_dtheta = (theta_rad - prev_theta) / Ts;
    theta_dot = (1.0f - alpha_d) * raw_dtheta + alpha_d * theta_dot;
    prev_theta = theta_rad;

    // 3) Estimasi posisi & kecepatan cart dari stepper
    x_m = (step_count - x_center_steps) / STEPS_PER_METER;
    static float prev_x = 0.0f;
    v_mps = (x_m - prev_x) / Ts;
    prev_x = x_m;

    // 4) LQR kontrol
    float u = -(Kx * x_m + Kv * v_mps + Kt * theta_rad + Ktd * theta_dot); // Newton (abstrak)

    // 5) Konversi ke step rate
    float v_cmd = U_TO_V * u;                       // m/s
    target_step_rate = V_TO_STEPRATE * v_cmd;       // steps/s

    // 6) Batasan dan proteksi
    float MAX_RATE = 20000.0f; // sesuaikan batas TB6600/NEMA17
    if (target_step_rate >  MAX_RATE) target_step_rate =  MAX_RATE;
    if (target_step_rate < -MAX_RATE) target_step_rate = -MAX_RATE;

    // Limit switch stop
    if (!digitalRead(PIN_LS_L) && target_step_rate < 0) target_step_rate = 0;
    if (!digitalRead(PIN_LS_R) && target_step_rate > 0) target_step_rate = 0;

    // 7) Set arah
    if (target_step_rate >= 0) {
      dir_cmd = DIR_RIGHT;
    } else {
      dir_cmd = DIR_LEFT;
    }
    digitalWrite(PIN_DIR, dir_cmd);
  }

  // 8) Step generation non-blocking berdasarkan target_step_rate
  stepService();
}

// -------------------- HOMING ----------------------------
void homing() {
  // Gerak ke kiri sampai limit kiri, lalu ke kanan sedikit, set center
  digitalWrite(PIN_DIR, DIR_LEFT);
  while (digitalRead(PIN_LS_L)) {
    pulseStep(6000); // 6000 steps/s
  }
  step_count = 0;
  delay(200);
  // Pergi ke kanan jarak tertentu, ambil sebagai center
  moveSteps(20000, DIR_RIGHT, 6000);
  x_center_steps = step_count;
  homed = true;
}

// -------------------- STEP SERVICE ----------------------
void stepService() {
  if (target_step_rate == 0.0f) return;
  unsigned long now = micros();
  unsigned long period = (unsigned long)(1000000.0f / fabs(target_step_rate));
  if (now - last_step_us >= period) {
    last_step_us = now;
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_STEP, LOW);
    // update counter
    if (dir_cmd == DIR_RIGHT) step_count++;
    else step_count--;
  }
}

void moveSteps(long steps, int dir, float rate) {
  digitalWrite(PIN_DIR, dir);
  unsigned long period = (unsigned long)(1000000.0f / rate);
  for (long i = 0; i < steps; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(period);
    if (dir == DIR_RIGHT) step_count++;
    else step_count--;
  }
}

void pulseStep(float rate) {
  static unsigned long last = 0;
  unsigned long now = micros();
  unsigned long period = (unsigned long)(1000000.0f / rate);
  if (now - last >= period) {
    last = now;
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_STEP, LOW);
  }
}

// -------------------- AS5600 ----------------------------
float readAS5600Radians() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_H);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0.0f;
  uint16_t high = Wire.read();
  uint16_t low = Wire.read();
  uint16_t angle12 = ((high & 0x0F) << 8) | low; // 12-bit
  float angle = (float)angle12 * (2.0f * PI / 4096.0f); // 0..2pi
  // Konversi ke -pi..pi dengan pi = tegak ke atas
  // Sesuaikan arah sesuai pemasangan sensor
  float theta = angle - PI; // kira-kira tegak
  return theta;
}

float wrapPi(float a) {
  while (a > PI)  a -= 2.0f*PI;
  while (a < -PI) a += 2.0f*PI;
  return a;
}