# Inverted Pendulum on a Cart (LQR)

## 1. Tujuan

Membangun sistem inverted pendulum on a cart. Komputasi gain LQR dilakukan di Python. Arduino menjalankan kontrol real-time. Motor menggerakkan cart menggunakan timing belt. Sudut pendulum dibaca menggunakan AS5600. Limit switch membatasi gerak di ujung rail.

## 2. Perangkat

* Arduino Uno
* Motor NEMA 17
* Driver TB6600
* Encoder AS5600
* Timing belt + pulley
* Limit switch kiri dan kanan
* Linear rail

## 3. Struktur Berkas

### 3.1 lqr_design.py

* Menghitung gain LQR diskrit (u = −Kx)
* Input: M, m, l, friction b (opsional), Ts
* Output: Kx, Kv, Kt, Ktd
* Q, R dapat disesuaikan
* Model linear upright untuk inverted pendulum
* Diskritisasi dengan Ts

### 3.2 InvertedPendulum_LQR.ino

* Loop kontrol 5 ms
* Pembacaan sudut AS5600
* Estimasi θ̇ dengan filter sederhana
* Estimasi x dan v dari step stepper
* Hitung u = −Kx
* Mapping u ke step rate
* Limit proteksi
* Homing

## 4. Parameter Fisik

* M: massa cart
* m: massa pendulum
* l: jarak pivot ke pusat massa
* Pitch timing belt (contoh GT2 = 2 mm)
* Teeth pulley (contoh 20T)
* Microstepping TB6600
* Motor steps per revolution (200)

## 5. Perhitungan STEPS_PER_METER

```
steps_per_meter = (motor_steps_rev × microstep) / (teeth × pitch_m)
```

Contoh GT2 2 mm, pulley 20T, microstep 16x:

```
(200 × 16) / (20 × 0.002) = 80000 steps/m
```

## 6. Kalibrasi Sudut Pendulum

1. Tegakkan pendulum
2. Baca θ dari AS5600
3. Set θ offset agar saat tegak sudut bernilai 0 rad

## 7. Desain LQR

1. State x = [x, v, θ, θ̇]
2. Bentuk A, B di Python
3. Pilih Q, R

   * Besarkan bobot θ dan θ̇ untuk menekankan stabilisasi
   * R lebih besar menjadikan kontrol lebih lembut
4. Diskritisasi menggunakan Ts = 0.005 s
5. Pecahkan DARE untuk dapatkan K
6. Masukkan K ke Arduino

## 8. Loop Kontrol Arduino

1. Baca θ
2. Hitung θ̇
3. Estimasi x, v berdasarkan step
4. Hitung u = −Kx
5. Konversi u menjadi step rate motor
6. Saturasi
7. Limit switch stop
8. Step service non-blocking

## 9. Homing

1. Gerak kiri sampai limit kiri aktif
2. step_count = 0
3. Geser kanan beberapa step
4. Simpan posisi sebagai center

## 10. Prosedur Running

1. Jalankan lqr_design.py
2. Simpan K = [Kx Kv Kt Ktd]
3. Isi K ke sketch Arduino
4. Hitung STEPS_PER_METER
5. Set θ offset
6. Uji gerak cart tanpa pendulum
7. Coba balancing dari posisi hampir tegak
8. Tuning Q dan R jika perlu

## 11. Tuning

* Overshoot → naikkan R
* Respon lambat → turunkan R atau naikkan bobot θ
* Terlalu sensitif → kecilkan bobot θ atau θ̇
* Mulai dengan U_TO_V kecil
* MAX_RATE rendah dulu

## 12. Troubleshooting

1. Cart tidak bergerak

   * Cek DIR
   * Cek wiring TB6600
   * U_TO_V terlalu kecil
2. Goyang cepat

   * Besarkan R
   * Kecilkan bobot θ̇
3. Pendulum jatuh ke samping

   * Offset θ salah
   * Rail miring
4. Tidak stabil

   * Sampling >5 ms
   * Gesekan besar

## 13. Tips

* Hindari delay
* Gunakan micros
* Pastikan limit aktif
* Tes tanpa pendulum dulu
* Jaga kebersihan kabel AS5600

## 14. Opsi

* Tambah LQI untuk tracking x_ref
* Filter tambahan untuk θ̇
* Logging serial untuk tuning

## 15. Catatan

* Struktur dan langkah mengikuti versi komprehensif
* Penyajian disusun seperti versi ringkas dengan heading
