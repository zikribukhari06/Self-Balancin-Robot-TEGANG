/*
  TEGANG - Firmware REMOTE universal  (v1.0)
  Board  : ESP32 Dev Module (WROOM-32 / WROOM-32U), Arduino-ESP32 core 3.x
  Library: hanya bawaan core (WiFi, esp_now)

  Remote ini TIDAK tahu apa-apa soal robot: dia cuma mengirim nilai mentah
  (4 sumbu -1000..1000 + 4 switch). Arti sumbu/switch ditentukan di firmware penerima.

  Untuk TEGANG:  LY (stick kiri, maju+) = kecepatan maju/mundur
                 RX (stick kanan, kanan+) = belok
                 SW1 = ARM (OFF lalu ON)    SW2 = E-STOP (ON = berhenti)

  LANGKAH PERTAMA: upload robot_tegang, buka Serial Monitor robot, catat "MAC robot",
  lalu isi ROBOT_MAC di bawah. Kalau dibiarkan nol, remote mengirim broadcast
  (tanpa ACK/retry, hanya untuk uji sebentar).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "tegang_protocol.h"

// ====================== PIN REMOTE ======================
// Sumbu analog WAJIB pin ADC1 (GPIO 32-39). ADC2 mati saat WiFi/ESP-NOW aktif.
#define PIN_LX   32     // stick kiri X   (belum dipakai TEGANG)
#define PIN_LY   34     // stick kiri Y   (maju/mundur)
#define PIN_RX   35     // stick kanan X  (belok)
#define PIN_RY   33     // stick kanan Y  (belum dipakai TEGANG)
#define PIN_SW1  25
#define PIN_SW2  26
#define PIN_SW3  27
#define PIN_SW4  14
#define PIN_RBAT 36     // opsional: pembagi 100k/100k dari baterai remote
#define PIN_LED  2

// ====================== KONFIGURASI ======================
// MAC ESP32 ROBOT (lihat Serial Monitor robot saat boot). Nol semua = broadcast.
static uint8_t ROBOT_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Sumbu yang dipasang? 0 = tidak dibaca (nilai 0), supaya pin mengambang tidak mengirim noise.
static const bool USE_AXIS[4]    = {false, true, true, false};   // LX, LY, RX, RY
// Balik arah sumbu bila perlu. Default: stick ke DEPAN = ADC rendah (sesuai kode awal lu).
static const bool INVERT_AXIS[4] = {false, true, false, false};
static const int  DEADZONE_ADC   = 150;                          // di sekitar titik tengah
#define USE_RBAT 0                                                // 1 = ukur baterai remote di GPIO 36
#define SEND_INTERVAL_MS 20                                      // 50 Hz

// ====================== GLOBAL ======================
static const int AXIS_PIN[4] = {PIN_LX, PIN_LY, PIN_RX, PIN_RY};
static const int SW_PIN[4]   = {PIN_SW1, PIN_SW2, PIN_SW3, PIN_SW4};
int axisCenter[4] = {2048, 2048, 2048, 2048};
uint8_t swStable = 0;            // bit i = switch i+1 ON (sudah di-debounce)
uint8_t swCount[4] = {0, 0, 0, 0};
uint16_t seq = 0;
bool useBroadcast = false;

// status terakhir dari robot
RobotStatusPacket lastStatus;
volatile bool statusValid = false;
volatile uint32_t lastStatusMs = 0;

// ====================== UTILITAS ======================
static bool macAllZero(const uint8_t *m) {
  for (int i = 0; i < 6; i++) if (m[i]) return false;
  return true;
}
static int readAvg(int pin) {
  int s = 0;
  for (int i = 0; i < 4; i++) s += analogRead(pin);
  return s / 4;
}
// 0..4095 -> -1000..1000 dengan titik tengah hasil kalibrasi dan deadzone
static int16_t scaleAxis(int raw, int center, bool invert) {
  float v;
  if (raw > center + DEADZONE_ADC)      v = (raw - center - DEADZONE_ADC) / (float)(4095 - center - DEADZONE_ADC);
  else if (raw < center - DEADZONE_ADC) v = (raw - center + DEADZONE_ADC) / (float)(center - DEADZONE_ADC);
  else                                  v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  if (invert) v = -v;
  return (int16_t)(v * 1000.0f);
}
static void updateSwitches() {
  for (int i = 0; i < 4; i++) {
    bool raw = (digitalRead(SW_PIN[i]) == LOW);            // INPUT_PULLUP: ditekan/ON = LOW
    bool cur = (swStable >> i) & 1;
    if (raw == cur) { swCount[i] = 0; continue; }
    if (++swCount[i] >= 3) {                               // 3 x 20 ms stabil = sah
      if (raw) swStable |= (1 << i); else swStable &= ~(1 << i);
      swCount[i] = 0;
    }
  }
}

// ====================== ESP-NOW ======================
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(RobotStatusPacket)) return;
  RobotStatusPacket s;
  memcpy(&s, data, sizeof(s));
  if (s.ver != PKT_VER_STATUS) return;
  if (s.crc != tegangCrc8((const uint8_t *)&s, sizeof(s) - 1)) return;
  lastStatus = s;
  lastStatusMs = millis();
  statusValid = true;
}

void calibrateSticks() {
  Serial.println("Kalibrasi joystick: JANGAN SENTUH stick selama 1 detik...");
  for (int i = 0; i < 4; i++) {
    if (!USE_AXIS[i]) continue;
    long sum = 0;
    for (int k = 0; k < 64; k++) { sum += analogRead(AXIS_PIN[i]); delay(2); }
    axisCenter[i] = (int)(sum / 64);
    Serial.printf("  sumbu %d titik tengah = %d\n", i, axisCenter[i]);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== TEGANG remote v1.0 ===");
  pinMode(PIN_LED, OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(SW_PIN[i], INPUT_PULLUP);
  for (int i = 0; i < 4; i++) if (USE_AXIS[i]) pinMode(AXIS_PIN[i], INPUT);
  calibrateSticks();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.print("MAC remote (isi ke REMOTE_MAC di robot bila mau filter): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW GAGAL inisialisasi");
    for (;;) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }
  esp_now_register_recv_cb(onRecv);

  useBroadcast = macAllZero(ROBOT_MAC);
  if (useBroadcast) {
    memset(ROBOT_MAC, 0xFF, 6);
    Serial.println("PERINGATAN: ROBOT_MAC belum diisi -> broadcast (tanpa ACK). Isi MAC robot.");
  }
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, ROBOT_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Gagal menambah peer");
    for (;;) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }
  Serial.println("Siap. SW1 OFF lalu ON untuk ARM robot. SW2 ON = E-STOP.");
}

void loop() {
  static uint32_t lastSend = 0, lastPrint = 0;
  uint32_t now = millis();

  // LED: dari status robot
  if (statusValid && now - lastStatusMs < 1000) {
    switch (lastStatus.state) {
      case RS_ARMED:    digitalWrite(PIN_LED, HIGH); break;
      case RS_FAULT:    digitalWrite(PIN_LED, ((now / 100) % 2) == 0); break;
      default:          digitalWrite(PIN_LED, (now % 1000) < 100); break;
    }
  } else {
    digitalWrite(PIN_LED, ((now / 500) % 2) == 0);          // belum ada status robot: kedip lambat
  }

  if (now - lastSend < SEND_INTERVAL_MS) return;
  lastSend = now;

  updateSwitches();

  RemotePacket p;
  memset(&p, 0, sizeof(p));
  p.ver = PKT_VER_REMOTE;
  p.buttons = swStable & 0x0F;
  p.seq = seq++;
  for (int i = 0; i < 4; i++) {
    p.axes[i] = USE_AXIS[i] ? scaleAxis(readAvg(AXIS_PIN[i]), axisCenter[i], INVERT_AXIS[i]) : 0;
  }
#if USE_RBAT
  float rv = analogReadMilliVolts(PIN_RBAT) * 2.0f / 1000.0f;     // pembagi 100k/100k
  p.rbat_dV = (uint8_t)(rv * 10.0f);
#else
  p.rbat_dV = 0;
#endif
  p.crc = tegangCrc8((const uint8_t *)&p, sizeof(p) - 1);
  esp_now_send(ROBOT_MAC, (const uint8_t *)&p, sizeof(p));

  if (now - lastPrint >= 500) {                                   // log 2 Hz
    lastPrint = now;
    Serial.printf("TX LX=%d LY=%d RX=%d RY=%d SW=%d%d%d%d", p.axes[0], p.axes[1], p.axes[2], p.axes[3],
                  (p.buttons & 1) != 0, (p.buttons & 2) != 0, (p.buttons & 4) != 0, (p.buttons & 8) != 0);
    if (statusValid && now - lastStatusMs < 1000) {
      Serial.printf(" | ROBOT %s fault=%s vbat=%.2fV pitch=%.1f%s%s\n",
                    stateName(lastStatus.state), faultName(lastStatus.fault),
                    lastStatus.vbat_cV / 100.0f, lastStatus.pitch_cdeg / 100.0f,
                    (lastStatus.flags & FLG_BATT_WARN) ? " BATT-LEMAH" : "",
                    (lastStatus.flags & FLG_TUNING) ? " TUNING" : "");
    } else {
      Serial.println(" | robot: tidak ada status");
    }
  }
}
