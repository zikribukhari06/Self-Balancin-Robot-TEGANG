# TEGANG: Two-Wheeled Self-Balancing Robot

![ESP32](https://img.shields.io/badge/ESP32-Dual%20Core-blue)
![Control](https://img.shields.io/badge/Control-Cascaded%20PID-green)
![Status](https://img.shields.io/badge/Status-Active-brightgreen)

**TEGANG** adalah proyek robot beroda dua (*self-balancing*) berbasis mikrokontroler ESP32 yang dikembangkan dengan arsitektur mekanis vertikal 4 lantai. Robot ini memanfaatkan sistem kendali presisi tinggi untuk menjaga keseimbangan secara dinamis, mengintegrasikan komunikasi nirkabel latensi rendah, serta memiliki fitur keselamatan otonom berbasis jarak.

---

## 🛠️ Fitur Utama

- **Dual-Core Processing:** Memaksimalkan arsitektur ESP32 (FreeRTOS). Core 1 didedikasikan penuh untuk komputasi kendali PID pada frekuensi tetap 100Hz, sedangkan Core 0 menangani jaringan WiFi dan ESP-NOW tanpa mengganggu *timing* motor.
- **Cascaded PID Control:** Menggunakan dua lapis algoritma PID. *Inner Loop* membaca sudut kemiringan (MPU6050), sementara *Outer Loop* membaca kecepatan dan posisi (Encoder Motor JGA25) untuk mencegah robot melayang (*drifting*).
- **Real-Time Web Tuning (SoftAP):** Kalibrasi parameter Kp, Ki, dan Kd (baik Inner maupun Outer) dapat dilakukan langsung melalui antarmuka Web UI dari *smartphone* tanpa perlu melakukan *re-flash* firmware.
- **ESP-NOW Remote Control:** Dikendalikan secara nirkabel menggunakan *remote* kustom berbasis ESP32 dengan latensi sangat rendah.
- **Safety Auto-Braking:** Integrasi dua sensor HC-SR04P (depan & belakang) yang menimpa (*override*) perintah *remote* dan memaksa robot berhenti/tegak jika mendeteksi halangan pada jarak $\le$ 15 cm.

---

## 🏗️ Arsitektur Mekanis & Perangkat Keras

Desain fisik dibagi menjadi 4 lantai menggunakan *spacer* baja 6cm untuk memanipulasi *Center of Gravity* (CoG) ke titik yang lebih tinggi, sehingga memperlambat inersia jatuh (*inverted pendulum physics*):

- **Layer 1 (Lantai Dasar):** 2x Motor DC Gearbox JGA25-370 (12V, 280 RPM, Encoder) dan 2x Driver Motor TB6612FNG (Sistem pin PWM dan IN diparalel ganda untuk menangani arus motor yang besar secara aman).
- **Layer 2:** Manajemen Daya terpusat berisi BMS 3S 60A dan Step-down/Buck Converter LM2596 (Output 5V untuk logika).
- **Layer 3:** Holder dan Baterai Li-ion 18650 3S 12.6V (Ditempatkan di tengah atas untuk distribusi bobot utama).
- **Layer 4 (Lantai Atas):** Otak sistem (ESP32), Sensor Inertia (MPU6050), dan 2x Sensor Ultrasonik HC-SR04P menghadap depan-belakang.

---

## ⚙️ Cara Kerja Sistem (How It Works)

Keseluruhan sistem keseimbangan TEGANG bekerja dalam siklus algoritma umpan balik (*feedback loop*) yang sangat cepat:

1. **Pembacaan Inertia (Sensor Fusion):** MPU6050 membaca data *Accelerometer* dan *Gyroscope*. Data ini diproses menggunakan filter matematis untuk mendapatkan sudut kemiringan (*Pitch*) aktual robot secara *real-time*.
2. **Kalkulasi Posisi (Outer PID - 20Hz):** Mikrokontroler membaca pulsa dari *encoder* di roda kiri dan kanan melalui *hardware interrupt*. Outer PID mengevaluasi seberapa jauh robot bergeser dari posisinya dan merumuskan nilai `pitch_offset` (kompensasi sudut).
3. **Kalkulasi Keseimbangan (Inner PID - 100Hz):** Nilai `pitch_offset` dari Outer Loop dan perintah maju/mundur dari *Remote* ESP-NOW digabungkan menjadi `target_pitch`. Inner PID menghitung selisih antara sudut target dan sudut aktual, lalu menghasilkan nilai P-I-D gabungan.
4. **Aktuasi Motor:** Nilai akhir dari PID diterjemahkan menjadi sinyal PWM (0-255) dan logika arah (High/Low) yang dikirim ke driver TB6612FNG. Jika robot condong ke depan, roda akan berputar maju ke bawah pusat massa untuk menahannya agar tidak jatuh.
5. **Mode Toggle:** Tekan tombol `BOOT` pada ESP32 untuk berpindah dari mode operasional (ESP-NOW) ke mode kalibrasi (Web Server Tuning).

---

## 🚀 Panduan Instalasi & Penggunaan

### 1. Kebutuhan Perangkat Lunak (Libraries)
Pastikan Anda telah menginstal pustaka berikut di Arduino IDE:
- `MPU6050_light` (Untuk pembacaan sensor gerak)
- Pustaka bawaan ESP32: `WiFi.h`, `esp_now.h`, `WebServer.h`, `Wire.h`

### 2. Kalibrasi Awal
1. Nyalakan robot dan pastikan memegangnya dalam posisi **benar-benar tegak 90 derajat** selama 3 detik pertama (*booting*) agar MPU6050 dapat merekam *zero-offset*.
2. Tekan tombol `BOOT` pada ESP32 untuk mengaktifkan SoftAP.
3. Hubungkan *smartphone*/PC ke WiFi `TEGANG_Tuning` (Password: `12345678`).
4. Buka browser dan akses `http://192.168.4.1` untuk mulai mencari nilai Kp, Ki, dan Kd yang tepat.

### 3. Safety Cut-Off
Untuk melindungi mekanik motor, aktuator akan otomatis dimatikan sepenuhnya (PWM = 0) apabila kemiringan robot melewati batas kritis $> 45^\circ$.
