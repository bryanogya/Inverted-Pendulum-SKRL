#include <Adafruit_AS5600.h>

// === Konfigurasi Stepper Motor ===
#define directionPin 2      // Pin arah (DIR)
#define stepPin 3           // Pin pulsa (PUL)
#define limitSwitchPinKa 6    // Pin limit switch (titik nol di kanan)
#define limitSwitchPinKi 7    // Pin limit switch (titik nol di kiri)
#define stepsPerRevolution 6400
#define distancePerRevolution 37.7  // mm per putaran

float stepsPerMM = stepsPerRevolution / distancePerRevolution;
float posisiSekarang = 0.0;   // posisi relatif (mm), 0 = titik nol
float kecepatanSekarang = 0.0;

// === Batas Area Kerja ===
float batasKiri = -200.0;     // mm
float batasKanan = 0.0;       // nol = limit switch

// === Sensor Sudut AS5600 ===
Adafruit_AS5600 as5600;
float offsetSudut = 0.0;      // offset untuk menjadikan sudut awal = 0°

// === Parameter PID ===
float Kp = 1.5;
float Ki = 0.02;
float Kd = 0.25;

float error = 0, lastError = 0;
float integral = 0;
float setpoint = 0.0;   // sudut tegak (derajat)

// === Kecepatan maksimum cart ===
float kecepatanMax = 100.0;  // mm/s

// === Waktu sampling PID ===
unsigned long lastTime = 0;
float dt = 0.01; // 10 ms

// === Setup ===
void setup() {
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limitSwitchPinKa, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== Sistem Pendulum on Cart (Area Kiri, Sudut Awal = 0°) ===");

  // Inisialisasi sensor AS5600
  if (!as5600.begin()) {
    Serial.println("Gagal mendeteksi AS5600! Periksa wiring.");
    while (1) delay(10);
  }
  Serial.println("AS5600 terdeteksi.");

  testSwitch();  // Set titik nol cart

  // 🔹 Ambil pembacaan awal sudut dan jadikan 0°
  offsetSudut = (as5600.getRawAngle() * 360.0) / 4096.0;
  Serial.print("Kalibrasi sudut awal: ");
  Serial.print(offsetSudut, 2);
  Serial.println("° dijadikan 0°");
  delay(1000);
}

// === Loop utama ===
void loop() {
  PID();
}

void PID(){
  unsigned long now = millis();
  if (now - lastTime >= 10) {  // eksekusi setiap 10 ms
    lastTime = now;

    // 1️⃣ Baca sudut dari AS5600 (sudah dikoreksi offset)
    float sudut = bacaSudut();

    // 2️⃣ Hitung error (setpoint = 0 derajat)
    error = setpoint - sudut;
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    lastError = error;

    // 3️⃣ Hitung keluaran PID (kecepatan cart)
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Batasi output
    output = constrain(output, -kecepatanMax, kecepatanMax);

    // 4️⃣ Gerakkan cart sesuai output PID
    if (output > 0) {  
      // Bergerak ke kanan (tidak boleh lewat nol)
      if (posisiSekarang < batasKanan)
        moveMotor(true, output * dt);  // jarak = kecepatan * waktu
    } 
    else if (output < 0) {
      // Bergerak ke kiri (tidak boleh lewat batas kiri)
      if (posisiSekarang > batasKiri)
        moveMotor(false, -output * dt);
    }

    // 5️⃣ Tampilkan status
    Serial.print("Sudut: ");
    Serial.print(sudut, 2);
    Serial.print("° | Error: ");
    Serial.print(error, 2);
    Serial.print(" | Output: ");
    Serial.print(output, 2);
    Serial.print(" | Posisi: ");
    Serial.print(posisiSekarang, 2);
    Serial.println(" mm");
  }
}

// === Fungsi Baca Sudut dari AS5600 ===
float bacaSudut() {
  if (!as5600.isMagnetDetected()) {
    Serial.println("Magnet tidak terdeteksi!");
    return 0;
  }

  uint16_t raw = as5600.getRawAngle();
  float angle = (raw * 360.0) / 4096.0;

  //kalibrasi sensor jadi 0 derajat ke bawah (alfa)
  angle = angle - offsetSudut;
  if (angle < 0) angle += 360.0;

  return angle;
}

//Fungsi untuk test switch
void testSwitch(){
  while(digitalRead(limitSwitchPinKa) == HIGH){
    Serial.println("Switch Kanan dipencet");
    delay(100);
  }

  while(digitalRead(limitSwitchPinKi) == HIGH){
    Serial.println("Switch Kiri dipencet");
    delay(100);
  }

  if(limitSwitchPinKa == HIGH &&  limitSwitchPinKi == HIGH){
    Serial.println("kedua switch dapat digunakan, stepper aman dijalankan");
    doHoming();
  }
}

// === Fungsi Homing (titik nol di kanan) ===
void doHoming() {
  Serial.println("Homing ke kanan...");
  digitalWrite(directionPin, LOW); // arah kanan
  
  while (digitalRead(limitSwitchPinKa) == HIGH) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
  }

  posisiSekarang = 0;
  Serial.println("Limit switch tertekan, posisi = 0 mm");
  delay(500);
}

// === Fungsi Gerak Motor (arahKanan=true → kanan) ===
void moveMotor(bool arahKanan, float jarakMM) {
  if (jarakMM <= 0) return;

  long langkah = jarakMM * stepsPerMM;
  digitalWrite(directionPin, arahKanan ? LOW : HIGH);

  for (long i = 0; i < langkah; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
  }

  posisiSekarang += (arahKanan ? jarakMM : -jarakMM);
}
