
# Hitung gain LQR (kontinu dan diskrit) untuk inverted pendulum on a cart
# Jalankan di PC/laptop. Setelah dapat K, copy ke sketch Arduino.
# Pastikan memasang paket: numpy, scipy (opsional). Versi tanpa scipy disediakan.

import numpy as np

# -------------------- PARAMETER FISIK --------------------
# Ganti sesuai hardware
M = 0.8     # massa cart [kg]
m = 0.2     # massa pendulum [kg]
l = 0.25    # jarak pusat massa ke pivot [m]
g = 9.81
b = 0.0     # friksi cart (opsional)
J = (1/3)*m*(2*l)**2  # momen inersia batang sekitar pivot (pendulum panjang 2*l)

# Model standar (linearized upright) dengan state x = [x, x_dot, theta, theta_dot]
# Dinamika kontinu (lihat referensi model klasik):
# A, B di-derive dari persamaan gerak. Di sini bentuk umum:

den = (M+m) * (J + m*l**2) - (m*l)**2
A = np.array([
    [0, 1, 0, 0],
    [0, -(b*(J+m*l**2))/den, (m**2*g*l**2)/den, 0],
    [0, 0, 0, 1],
    [0, -(b*m*l)/den, (m*g*l*(M+m))/den, 0]
], dtype=float)
B = np.array([
    [0],
    [(J+m*l**2)/den],
    [0],
    [m*l/den]
], dtype=float)

# -------------------- PENIMBANG Q,R --------------------
# Atur penalti. Besarkan bobot pada theta agar tegak.
Q = np.diag([0.1, 0.0, 80.0, 2.0])
R = np.array([[0.02]])

# -------------------- DISKRETISASI ----------------------
Ts = 0.005  # 5 ms loop pada Arduino

# c2d (Euler atau bilinear). Dipakai eksponensial blok untuk akurat.
def c2d(A,B,Ts):
    n = A.shape[0]
    M = np.block([
        [A, B],
        [np.zeros((1, n+1))]
    ])
    Md = np.linalg.expm(M*Ts)
    Ad = Md[:n,:n]
    Bd = Md[:n,n:n+1]
    return Ad, Bd

# Fallback jika np.linalg.expm tidak ada: gunakan pade dari scipy atau euler
try:
    Ad, Bd = c2d(A,B,Ts)
except Exception:
    Ad = np.eye(A.shape[0]) + A*Ts
    Bd = B*Ts

# -------------------- LQR DISKRET -----------------------
# Pecahkan DARE untuk mendapatkan K

def dare(A,B,Q,R,eps=1e-9, maxit=10000):
    P = Q.copy()
    for _ in range(maxit):
        Pn = A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
        if np.max(np.abs(Pn-P)) < eps:
            P = Pn
            break
        P = Pn
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    return K, P

K, P = dare(Ad, Bd, Q, R)

print("Gain K (u = -K x) =", K)
print("Ad =\n", Ad)
print("Bd =\n", Bd)
print("Ts =", Ts)

# Skala kecepatan stepper
# Anda perlu mapping: u [N] -> v_cart [m/s] -> step_rate [steps/s]
# Ini tergantung rasio mekanik. Isi setelah kalibrasi.



# Catatan pemakaian

# 1. Jalankan lqr_design.py. Isi M, m, l, b, dan Ts. Simpan K = [Kx Kv Kt Ktd].

# 2. Masukkan K ke sketch Arduino pada bagian LQR GAINS.

# 3. Kalibrasi THETA_OFFSET. Pegang pendulum tegak. Cetak theta saat tegak. Set nilai offset agar 0 rad saat tegak.

# 4. Hitung STEPS_PER_METER untuk timing belt. Rumus: steps_per_meter = (motor_steps_per_rev × microstep) / (teeth × pitch_m). Contoh GT2, pitch 2 mm, pulley 20T, microstep 16x. motor_steps_per_rev = 200. Maka steps_per_meter = (200×16) / (20×0.002) = 80000 steps/m.

# 5. Uji tanpa pendulum dulu untuk memastikan gerak kanan-kiri dan limit switch bekerja.

# 6. Saat balancing, mulai dari posisi mendekati tegak. Dorong ringan, lihat respon. Sesuaikan Q,R jika overshoot atau lambat.
