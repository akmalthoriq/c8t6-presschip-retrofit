# c8t6-presschip-retrofit

# ⚙️ Automatic Hydraulic Pressing & Auger Feeder Controller

![STM32 HAL](https://img.shields.io/badge/STM32-HAL_Driver-blue?style=for-the-badge&logo=stmicroelectronics)
![Architecture](https://img.shields.io/badge/Architecture-ARM_Cortex--M3-orange?style=for-the-badge)
![Interface](https://img.shields.io/badge/Interface-USB_VCP_(CDC)-green?style=for-the-badge)
![Language](https://img.shields.io/badge/Language-C-brightgreen?style=for-the-badge)

Firmware kontrol berbasis mikrokontroler **STM32F103** untuk mengendalikan **Mesin Press Hidrolik / Briket Otomatis** yang dilengkapi dengan *Pengisi Material Auger Motor*, *Silinder Press Vertikal*, *Pintu Pembuangan Horizontal*, serta *Pressure Transducer Sensor*.

---

## 📑 Daftar Isi
- [Fitur Utama](#-fitur-utama)
- [Wiring & Pinout](#-wiring--pinout)
  - [1. Sensor & Input Analog](#1-sensor--input-analog)
  - [2. Tombol Operasi & Selector](#2-tombol-operasi--selector)
  - [3. Output Relay (Aktuator)](#3-output-relay-aktuator)
- [Cara Kerja Firmware (State Machine)](#-cara-kerja-firmware-state-machine)
  - [Diagram Alur Siklus Otomatis](#diagram-alur-siklus-otomatis)
  - [Mode Siklus Press (Selector Switch)](#mode-siklus-press-selector-switch)
  - [Sistem Pengaman & Safety](#sistem-pengaman--safety)
- [Konversi Tekanan (Bar ke ADC)](#-konversi-tekanan-bar-ke-adc)
- [Antarmuka USB VCP (Virtual COM Port)](#-antarmuka-usb-vcp-virtual-com-port)
  - [Daftar Perintah Serial Command](#daftar-perintah-serial-command)
  - [Contoh Status Monitoring](#contoh-status-monitoring)
- [Penyimpanan Parameter (Flash Memory Persistence)](#-penyimpanan-parameter-flash-memory-persistence)
- [Cara Penggunaan](#-cara-penggunaan)
  - [1. Mode Manual](#1-mode-manual)
  - [2. Mode Otomatis](#2-mode-otomatis)

---

## 🚀 Fitur Utama

- **Dual Operating Mode**:
  - **Mode Manual**: Kontrol penuh setiap gerakan aktuator melalui tombol fisik terpisah dengan sistem *interlock safety*.
  - **Mode Otomatis**: Siklus otomatis lengkap menggunakan *Finite State Machine (FSM)*.
- **Multi-Cycle Selectable Press**: Pilihan siklus pemadatan (1x, 2x, atau 3x press) menggunakan *Selector Switch*.
- **Precision Pressure Monitoring**: Pembacaan tekanan sistem hidrolik via **ADC 12-bit DMA** (0–300 Bar) untuk *final press* dan *eject control*.
- **USB CDC / Virtual COM Port (VCP)**:
  - Monitoring status real-time (*State, Pressure, Sensor Proximity, Delay*).
  - Konfigurasi parameter sistem secara langsung via Terminal Serial.
- **Flash Memory Data Persistence**: Parameter konfigurasi disimpan secara permanen di Flash Memory Internal STM32 (Page 31) sehingga tidak hilang saat mati listrik.
- **Comprehensive Safety Systems**:
  - *Interlock Tombol Manual* (Mencegah dua arah aktuator berlawanan aktif bersamaan).
  - *Inconsistent Sensor Detection* (Emergency Stop jika kombinasi sensor tidak valid).
  - *Global & Gauge Timeouts* (Mencegah mesin terjebak/stuck akibat kegagalan mekanik/sensor).

---

## 📌 Wiring & Pinout

### 1. Sensor & Input Analog

| Nama Komponen | Pin STM32 | Tipe Pin | Aktivasi | Keterangan |
| :--- | :---: | :---: | :---: | :--- |
| **Pressure Gauge** | `PA0` | ADC1_IN0 | Analog (0-3.3V) | Transducer Tekanan Hidrolik (0 - 300 Bar) |
| **Prox Homing** | `PA4` | GPIO Input | Active LOW | Sensor batas atas silinder vertikal |
| **Prox Press / Mid** | `PA5` | GPIO Input | Active LOW | Sensor tengah silinder vertikal |
| **Prox Eject** | `PA6` | GPIO Input | Active LOW | Sensor batas bawah silinder vertikal |
| **Prox Close** | `PA7` | GPIO Input | Active LOW | Sensor pintu pembuangan horizontal rapat |
| **Prox Open** | `PB0` | GPIO Input | Active LOW | Sensor pintu pembuangan horizontal terbuka |

---

### 2. Tombol Operasi & Selector

| Nama Komponen | Pin STM32 | Tipe Pin | Pull Mode | Keterangan |
| :--- | :---: | :---: | :---: | :--- |
| **Selector Switch 1** | `PC13` | GPIO Input | Pull-Up | Bit-1 Pemilih Siklus Otomatis |
| **Selector Switch 2** | `PC14` | GPIO Input | Pull-Up | Bit-2 Pemilih Siklus Otomatis |
| **Btn Auger Mundur**| `PA9` | GPIO Input | Pull-Up | Tombol manual pemutar auger mundur |
| **Btn Auger Maju** | `PA10` | GPIO Input | Pull-Up | Tombol manual pemutar auger maju |
| **Btn Horiz Mundur**| `PB3` | GPIO Input | Pull-Up | Tombol manual pembuka pintu pembuangan |
| **Btn Horiz Maju** | `PB4` | GPIO Input | Pull-Up | Tombol manual penutup pintu pembuangan |
| **Btn Vert Turun** | `PB5` | GPIO Input | Pull-Up | Tombol manual silinder piston maju |
| **Btn Vert Naik** | `PB6` | GPIO Input | Pull-Up | Tombol manual silinder piston mundur |
| **Btn Auto Cycle** | `PB7` | GPIO Input | Pull-Up | Sakelar Latching pemilih Auto / Manual mode |

---

### 3. Output Relay (Aktuator)

> **Catatan Logika Output:** Relai diaktifkan dengan logika `HIGH` (`ACTUATOR_ON = GPIO_PIN_SET`) dan dimatikan dengan `LOW` (`ACTUATOR_OFF = GPIO_PIN_RESET`).

| Nama Aktuator | Pin STM32 | Tipe Output | Keterangan |
| :--- | :---: | :---: | :--- |
| **Relay Auger Maju** | `PB1` | Push-Pull Output | Menggerakkan motor auger pengisi bahan |
| **Relay Auger Mundur**| `PB10` | Push-Pull Output | Menggerakkan motor auger arah sebaliknya |
| **Relay Horiz Close** | `PB12` | Push-Pull Output | Katup Solenoid pintu pembuangan maju (tutup) |
| **Relay Horiz Open** | `PB13` | Push-Pull Output | Katup Solenoid pintu pembuangan mundur (buka) |
| **Relay Vert Down** | `PB14` | Push-Pull Output | Katup Solenoid silinder piston maju (press/eject) |
| **Relay Vert Up** | `PB15` | Push-Pull Output | Katup Solenoid silinder piston mundur (homing) |

---

## 🔄 Cara Kerja Firmware (State Machine)

Firmware bekerja berdasarkan arsitektur **Finite State Machine (FSM)** yang dieksekusi secara periodik pada fungsi `System_Run_Cycle()`.

### Diagram Alur Siklus Otomatis

```text
[START AUTO CYCLE]
       │
       ▼
[STATE_HORIZ_CLOSE_FIRST] ──► Menutup pintu pembuangan hingga PROX_CLOSE Aktif
       │
       ▼
[STATE_HOMING] ─────────────► Silinder piston mundur hingga PROX_HOMING Aktif
       │
       ▼
[STATE_AUGER_FILLING] ──────► Motor Auger mengisi chip/material selama (Delay Material In)
       │
       ├───► [Siklus awal (Mode 2/3)] ──► [STATE_VERT_PRESS_BY_TIMER] ──► Kembali ke HOMING
       │
       └───► [Siklus Akhir / Mode 1] ──► [STATE_VERT_PRESS_BY_GAUGE] (Press hingga Target Bar)
                                                      │
                                                      ▼
[STATE_HORIZ_OPEN] ◄──────── [STATE_VERT_UP_BY_TIMER] (piston mundur sejenak lepas tekanan)
       │
       ▼
[STATE_VERT_EJECT] ─────────► Piston mendorong briket keluar hingga terdeteksi 50 Bar
       │
       ▼
[STATE_HORIZ_CLOSE] ────────► Pintu pembuangan menutup rapat sambil mempertahankan 50 Bar
       │
       └────────────────────► Kembali ke [STATE_HOMING] untuk siklus berikutnya
```

---

### Mode Siklus Press (Selector Switch)

Mode ditentukan oleh posisi sakelar selector 3-posisi pada pin `PC13` dan `PC14`:

| PC13 (`SEL_SW_1`) | PC14 (`SEL_SW_2`) | Mode Siklus | Urutan Press |
| :---: | :---: | :---: | :--- |
| **LOW** | **HIGH** | **Mode 1** | 1x Press by Gauge (Final Press) |
| **HIGH** | **HIGH** | **Mode 2** | 1x Press by Timer $\rightarrow$ 1x Press by Gauge |
| **HIGH** | **LOW** | **Mode 3** | 2x Press by Timer $\rightarrow$ 1x Press by Gauge |

---

### Sistem Pengaman & Safety

Firmware secara otomatis berpindah ke `STATE_EMERGENCY_STOP` dan mematikan seluruh relay jika kondisi berikut terdeteksi:
1. **Konflik Sensor Vertikal**: `PROX_HOMING` dan `PROX_PRESS` aktif bersamaan.
2. **Konflik Sensor Horizontal**: `PROX_CLOSE` dan `PROX_OPEN` aktif bersamaan.
3. **Pressure Gauge Timeout**: Tekanan tidak mencapai target Bar dalam waktu 15 detik saat proses gauge press.
4. **Global State Timeout**: Terjebak dalam satu state otomatis lebih lama dari `global_timeout_duration_ms` (default 60 detik).

---

## 📐 Konversi Tekanan (Bar ke ADC)

Sistem menggunakan rumus kurva linier untuk mengonversi nilai bacaan sensor tekanan (0–300 Bar) ke skala ADC 12-bit (0–4095):

- **0 Bar** $\approx$ **410 ADC** (0.33V)
- **300 Bar** $\approx$ **3723 ADC** (3.00V)

$$\text{Slope} = \frac{3723 - 410}{300} = 11.0433$$

$$\text{ADC Target} = (\text{Slope} \times \text{Target Bar}) + 410$$

---

## 💻 Antarmuka USB VCP (Virtual COM Port)

Sistem dapat dipantau dan dikonfigurasi melalui koneksi USB CDC Serial Terminal (Baudrate: 115200 bps, 8N1).

### Daftar Perintah Serial Command

| Perintah Serial | Rentang Nilai | Fungsi |
| :--- | :--- | :--- |
| `$DMI:<ms>` | `500` – `60000` | Mengatur waktu pengisian auger (Delay Material In) dalam milidetik. |
| `$VUD:<ms>` | `500` – `10000` | Mengatur durasi silinder vertikal naik sementara sebelum eject. |
| `$PTD:<ms>` | `500` – `10000` | Mengatur durasi penekanan berbasis timer (Press By Timer). |
| `$GTO:<ms>` | `5000` – `60000` | Mengatur batas waktu timeout global per-state. |
| `$PTB:<bar>`| `50.0` – `300.0` | Mengatur target tekanan pemadatan akhir (Bar). |
| `GETSTATUS` | - | Meminta informasi status sistem secara real-time. |

---

### Contoh Status Monitoring

Mengirim perintah `GETSTATUS` melalui serial terminal akan menghasilkan respon seperti berikut:

```text
STATUS: IDLE | Auto:0 Mode Siklus: 1 | Press: 150.0 Bar | Delay:10000 ms | Proxies: H:1 M:0 E:0 C:1 O:0
```

---

## 💾 Penyimpanan Parameter (Flash Memory Persistence)

Setiap perubahan parameter via perintah USB Serial akan langsung disimpan ke dalam **Internal Flash Memory STM32F103** pada **Page 31** (`Address 0x08007C00`). 

Parameter yang tersimpan secara non-volatile meliputi:
1. `material_in_delay_ms`
2. `vert_up_duration_ms`
3. `press_by_timer_duration_ms`
4. `global_timeout_duration_ms`
5. `target_pressure_bar`

---

## 🛠️ Cara Penggunaan

### 1. Mode Manual
1. Pastikan Sakelar **Auto Cycle** (`PB7`) dalam posisi **OFF** (HIGH). Status sistem berada pada state `IDLE`.
2. Gunakan tombol fisik untuk melakukan pergerakan manual:
   - **Naik / Turun**: Menggerakkan silinder press vertikal.
   - **Maju / Mundur**: Membuka / menutup pintu pembuangan horizontal.
   - **Auger Maju / Mundur**: Memutar motor pengisi material.
3. Apabila dua tombol yang berlawanan ditekan bersamaan, sistem secara otomatis menghentikan pergerakan demi keamanan mekanis.

### 2. Mode Otomatis
1. Atur posisi **Selector Switch** (`PC13` & `PC14`) sesuai dengan mode siklus pemadatan yang diinginkan (Mode 1, 2, atau 3).
2. Nyalakan Sakelar **Auto Cycle** (`PB7`) ke posisi **ON** (LOW).
3. Mesin akan mengeksekusi urutan otomatis secara mandiri:
   - Menutup pintu pembuangan $\rightarrow$ Homing $\rightarrow$ Pengisian bahan via Auger $\rightarrow$ Pemadatan (Press) $\rightarrow$ Pembukaan Pintu $\rightarrow$ Pendorongan hasil (Eject) $\rightarrow$ Pengulangan Siklus.
4. Untuk menghentikan proses otomatis kapan saja, matikan Sakelar **Auto Cycle**. Sistem akan segera mematikan seluruh aktuator dan kembali ke mode `IDLE`.

---

## 📄 Lisensi & Hak Cipta

Hak Cipta (c) 2025 STMicroelectronics & Pengembang Sistem. Seluruh hak cipta dilindungi undang-undang.
