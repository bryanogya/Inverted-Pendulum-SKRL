# Inverted Pendulum on a Cart (PID)

## Tujuan

Sistem menyeimbangkan pendulum tetap tegak menggunakan kontrol PID berbasis Arduino Uno.

## Perangkat

* Arduino Uno
* NEMA17
* TB6600
* AS5600
* Timing belt + pulley
* Limit switch kiri dan kanan
* Linear rail

## File

* pid_main.ino

  * Loop kontrol 5 ms
  * PID sudut pendulum
  * Homing + limit + stepper

## Langkah Kerja

1. Ukur M, m, l, pitch, teeth, microstep.
2. Hitung steps_per_meter.
3. Kalibrasi sudut (θ=0 saat tegak).
4. Tuning PID.
5. Jalankan.

## Rumus steps_per_meter

(motorSteps × microstep) / (teeth × pitch_m)

## Tuning

* Overshoot: tambah Kd
* Lambat: tambah Kp
* Drift: tambahkan Ki kecil

## Troubleshooting

* Tidak stabil: sampling lambat
* Cart kabur: Ki besar
* Jatuh: offset salah
