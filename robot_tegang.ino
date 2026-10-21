/*
  TEGANG - Firmware ROBOT self-balancing  (v1.0)
  Board  : ESP32 Dev Module (WROOM-32), Arduino-ESP32 core 3.x
  Library: MPU6050_light, ESP32Encoder (>= 0.11.x), bawaan core: WiFi, esp_now, WebServer, Preferences

  Isi utama
  - Loop sudut 100 Hz (PID, D dari gyro) + loop kecepatan 20 Hz (encoder x4 / PCNT)
  - State machine: DISARMED -> ARMED -> FAULT (latched), arm lewat SW1, e-stop lewat SW2 di remote
  - Proteksi: clamp duty, slew-rate, deteksi stall, saturasi, runaway, jatuh +-45 derajat,
    baterai rendah, failsafe link (500 ms), normalisasi duty ke tegangan baterai
  - ESP-NOW (channel tetap, paket ber-CRC) + telemetri balik ke remote
  - Ultrasonik non-blocking (task di core 0) sebagai rem aman
  - Mode TUNING lewat web (tahan tombol BOOT 0,6 detik), parameter bisa disimpan ke flash

  KONVENSI TANDA (penting, cek di uji gantung):
    sudut (+)   = badan miring ke DEPAN
    perintah (+)= roda berputar MAJU
  Kalau arah salah, ubah PITCH_SIGN / MOTOR_x_SIGN / ENC_x_SIGN di bawah, JANGAN ubah rumus.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <MPU6050_light.h>
#include <ESP32Encoder.h>
#include "tegang_protocol.h"

// ====================== PIN (pinout final) ======================
#define PIN_BOOT     0
#define PIN_LED      2
#define PIN_PWM_L    13
#define PIN_IN1_L    4
#define PIN_IN2_L    14
#define PIN_PWM_R    25
#define PIN_IN1_R    26
#define PIN_IN2_R    27
#define PIN_STBY     15      // pull-down 10k di hardware
#define PIN_TRIG_F   5
#define PIN_ECHO_F   18
#define PIN_TRIG_B   19
#define PIN_ECHO_B   23
#define PIN_ENC_L_A  16
#define PIN_ENC_L_B  17
#define PIN_ENC_R_A  32
#define PIN_ENC_R_B  33
#define PIN_VBAT     35      // pembagi 47k/10k
#define PIN_SDA      21
#define PIN_SCL      22

// ====================== KONVENSI TANDA ======================
#define IMU_ANGLE()   (mpu.getAngleY())   // ganti ke getAngleX() kalau sumbu pitch ternyata X
#define IMU_RATE()    (mpu.getGyroY())    // ganti ke getGyroX() BERSAMAAN dengan di atas
#define PITCH_SIGN    (+1)   // balik jadi -1 kalau sudut + malah saat miring ke belakang
#define MOTOR_L_SIGN  (+1)   // balik kalau roda kiri mundur saat perintah +
#define MOTOR_R_SIGN  (-1)   // motor kanan terpasang berhadapan -> umumnya -1
#define ENC_L_SIGN    (+1)   // balik kalau count kiri negatif saat roda maju
#define ENC_R_SIGN    (-1)

// Kalau diisi (bukan nol semua), robot hanya menerima paket dari remote ini.
static const uint8_t REMOTE_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ====================== MEKANIK & KALIBRASI ======================
constexpr float WHEEL_D_CM    = 6.5f;      // diameter roda
constexpr float ENC_CPR_WHEEL = 940.0f;    // count per putaran roda (x4 x rasio ~21,3) -> UKUR di tes P1-11
constexpr float VBAT_DIV      = 5.7f;      // (47k+10k)/10k
constexpr float VBAT_CAL      = 1.00f;     // faktor kalibrasi vs multimeter

// ====================== PWM ======================
constexpr int PWM_FREQ = 20000;            // 20 kHz (tidak berdengung)
constexpr int PWM_BITS = 10;
constexpr int PWM_MAX  = (1 << PWM_BITS) - 1;

// ====================== BATAS KONTROL & PROTEKSI ======================
constexpr uint32_t LOOP_US          = 10000;   // 100 Hz
constexpr uint32_t OUTER_MS         = 50;      // 20 Hz loop kecepatan
constexpr float    FALL_DEG         = 45.0f;
constexpr float    ARM_MAX_ANGLE    = 3.0f;    // syarat arm: robot tegak +-3 derajat
constexpr float    ARM_MAX_RATE     = 10.0f;   // dan diam (deg/s)
constexpr float    I_LIMIT          = 30.0f;   // batas suku I (satuan 0..255)
constexpr float    THETA_REF_MAX    = 6.0f;    // batas sudut perintah dari loop kecepatan (derajat)
constexpr float    IV_LIMIT         = 3.0f;    // batas suku I loop kecepatan (derajat)
constexpr float    SLEW_U_PER_LOOP  = 51.0f;   // 20% skala per 10 ms
constexpr float    VREF_ACCEL       = 80.0f;   // cm/s^2 : ramp perintah kecepatan
constexpr float    YAW_SLEW_PER_S   = 400.0f;  // unit u per detik
constexpr float    STALL_U          = 80.0f;   // u di atas ini tapi roda diam = stall
constexpr float    STALL_V          = 1.5f;    // cm/s dianggap diam
constexpr int      STALL_WINDOWS    = 5;       // 5 x 50 ms = 250 ms
constexpr float    RUNAWAY_V        = 70.0f;   // cm/s
constexpr int      RUNAWAY_WINDOWS  = 10;      // 0,5 s
constexpr float    SAT_FAULT_MS     = 800.0f;  // output jenuh selama ini = fault
constexpr uint32_t LINK_DISARM_MS   = 3000;    // link putus selama ini saat ARMED -> DISARM
constexpr float    BATT_NOM_V       = 11.1f;
constexpr float    BATT_WARN_V      = 10.0f;
constexpr float    BATT_CUT_V       = 9.3f;
constexpr float    BATT_RECOVER_V   = 9.6f;
constexpr float    OBST_STOP_CM     = 25.0f;
constexpr float    OBST_CLEAR_CM    = 32.0f;

// ====================== PARAMETER (bisa diubah lewat web, disimpan ke flash) ======================
struct Params {
  float kp     = 15.0f;   // u per derajat
  float ki     = 1.0f;    // u per derajat-detik
  float kd     = 0.5f;    // u per (derajat/detik), dari gyro
  float kpv    = 0.10f;   // derajat per (cm/s)
  float kiv    = 0.02f;   // derajat per (cm/s * detik)
  float trim   = 0.0f;    // offset titik seimbang (derajat)
  float duty   = 0.50f;   // batas duty maksimum (0..0.9). MULAI 0,5
  float vmax   = 25.0f;   // kecepatan maksimum dari stick (cm/s)
  float yawmax = 40.0f;   // beda u maksimum untuk belok
} P;

// ====================== OBJEK GLOBAL ======================
MPU6050 mpu(Wire);
ESP32Encoder encL, encR;
WebServer server(80);
Preferences prefs;

enum Mode : uint8_t { MODE_NORMAL = 0, MODE_TUNING = 1 };
volatile Mode currentMode = MODE_NORMAL;
volatile bool apStarted = false;

uint8_t state = RS_DISARMED;
uint8_t fault = RF_NONE;

// data dari remote (ditulis callback di core 0, dibaca loop di core 1)
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
RemotePacket rxPkt;
volatile bool rxValid = false;
volatile uint32_t lastRxMs = 0;
uint8_t remoteMac[6] = {0};
volatile bool peerKnown = false;
volatile bool needAddPeer = false;
bool peerAdded = false;

struct Cmd { bool link; uint8_t buttons; int16_t axes[4]; };

// sensor jarak (diisi task core 0)
volatile float distF = 999.0f, distB = 999.0f;
bool obsF = false, obsB = false;

// baterai
float vbat = 0.0f;
bool vbatValid = false, battWarn = false, battCut = false, battInit = false;
uint8_t battCutCnt = 0;

// variabel kontrol
float g_pitch = 0, g_rate = 0, g_u = 0, g_v = 0;
float thetaRefF = 0, iTerm = 0, ivTerm = 0, vRef = 0, yawCmd = 0;
float uL = 0, uR = 0, satMs = 0;
float vAvg = 0, vL = 0, vR = 0;
long encPrevL = 0, encPrevR = 0;
int stallCntL = 0, stallCntR = 0, runCnt = 0;
uint32_t lastOuterMs = 0, lastLoopUs = 0;
bool telemetryOn = true;

// ====================== UTILITAS ======================
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float slewf(float cur, float target, float maxStep) {
  float d = target - cur;
  if (d > maxStep) d = maxStep;
  if (d < -maxStep) d = -maxStep;
  return cur + d;
}
static bool macAllZero(const uint8_t *m) {
  for (int i = 0; i < 6; i++) if (m[i]) return false;
  return true;
}
static float axisToUnit(int16_t a) {
  if (a > -40 && a < 40) return 0.0f;
  return clampf(a / 1000.0f, -1.0f, 1.0f);
}

// ====================== PARAMETER: SIMPAN / MUAT ======================
void loadParams() {
  prefs.begin("tegang", true);
  P.kp = prefs.getFloat("kp", P.kp);       P.ki = prefs.getFloat("ki", P.ki);
  P.kd = prefs.getFloat("kd", P.kd);       P.kpv = prefs.getFloat("kpv", P.kpv);
  P.kiv = prefs.getFloat("kiv", P.kiv);    P.trim = prefs.getFloat("trim", P.trim);
  P.duty = prefs.getFloat("duty", P.duty); P.vmax = prefs.getFloat("vmax", P.vmax);
  P.yawmax = prefs.getFloat("yaw", P.yawmax);
  prefs.end();
}
void saveParams() {
  prefs.begin("tegang", false);
  prefs.putFloat("kp", P.kp);     prefs.putFloat("ki", P.ki);
  prefs.putFloat("kd", P.kd);     prefs.putFloat("kpv", P.kpv);
  prefs.putFloat("kiv", P.kiv);   prefs.putFloat("trim", P.trim);
  prefs.putFloat("duty", P.duty); prefs.putFloat("vmax", P.vmax);
  prefs.putFloat("yaw", P.yawmax);
  prefs.end();
}
void printParams() {
  Serial.printf("PARAM kp=%.2f ki=%.2f kd=%.2f | kpv=%.3f kiv=%.3f | trim=%.2f duty=%.2f vmax=%.1f yaw=%.1f\n",
                P.kp, P.ki, P.kd, P.kpv, P.kiv, P.trim, P.duty, P.vmax, P.yawmax);
}

// ====================== MOTOR ======================
void motorsOff() {
  digitalWrite(PIN_STBY, LOW);
  ledcWrite(PIN_PWM_L, 0); ledcWrite(PIN_PWM_R, 0);
  digitalWrite(PIN_IN1_L, LOW); digitalWrite(PIN_IN2_L, LOW);
  digitalWrite(PIN_IN1_R, LOW); digitalWrite(PIN_IN2_R, LOW);
  uL = 0; uR = 0;
}

// u dalam -255..255, tanda (+) = maju (sebelum MOTOR_x_SIGN)
static void setMotor(int in1, int in2, int pwmPin, float u) {
  u = clampf(u, -255.0f, 255.0f);
  int duty = (int)(fabsf(u) / 255.0f * PWM_MAX);
  if (u > 0.0f)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (u < 0.0f) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else               { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  ledcWrite(pwmPin, duty);
}
void driveMotors(float left, float right) {
  digitalWrite(PIN_STBY, HIGH);
  setMotor(PIN_IN1_L, PIN_IN2_L, PIN_PWM_L, left * MOTOR_L_SIGN);
  setMotor(PIN_IN1_R, PIN_IN2_R, PIN_PWM_R, right * MOTOR_R_SIGN);
}

// ====================== BATERAI ======================
void readBattery() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 100) return;
  last = now;
  uint32_t mv = 0;
  for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(PIN_VBAT);
  float v = (mv / 4.0f) / 1000.0f * VBAT_DIV * VBAT_CAL;
  if (!battInit) { vbat = v; battInit = true; } else vbat += 0.1f * (v - vbat);   // EMA ~1 detik
  vbatValid = vbat > 5.0f;       // < 5 V = tidak ada baterai (mode meja/USB)
  if (!vbatValid) { battWarn = false; battCut = false; battCutCnt = 0; return; }
  if (vbat < BATT_CUT_V) { if (battCutCnt < 255) battCutCnt++; } else battCutCnt = 0;
  if (battCutCnt >= 10) battCut = true;          // 10 x 100 ms di bawah batas
  if (vbat > BATT_RECOVER_V) battCut = false;
  battWarn = vbat < BATT_WARN_V;
}

// ====================== ULTRASONIK (task core 0, tidak mengganggu loop kontrol) ======================
static float medianOf3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}
static float measureCm(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(3);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long d = pulseIn(echo, HIGH, 25000UL);     // timeout 25 ms (~4 m)
  if (d == 0) return 999.0f;
  return d * 0.0343f / 2.0f;
}
void sensorTask(void *) {
  float hf[3] = {999, 999, 999}, hb[3] = {999, 999, 999};
  int i = 0;
  for (;;) {
    hf[i] = measureCm(PIN_TRIG_F, PIN_ECHO_F);
    vTaskDelay(pdMS_TO_TICKS(35));                    // jeda anti cross-talk
    hb[i] = measureCm(PIN_TRIG_B, PIN_ECHO_B);
    i = (i + 1) % 3;
    distF = medianOf3(hf[0], hf[1], hf[2]);
    distB = medianOf3(hb[0], hb[1], hb[2]);
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}
void updateObstacle() {
  float f = distF, b = distB;
  if (f <= OBST_STOP_CM) obsF = true; else if (f >= OBST_CLEAR_CM) obsF = false;
  if (b <= OBST_STOP_CM) obsB = true; else if (b >= OBST_CLEAR_CM) obsB = false;
}

// ====================== ESP-NOW ======================
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(RemotePacket)) return;
  RemotePacket p;
  memcpy(&p, data, sizeof(p));
  if (p.ver != PKT_VER_REMOTE) return;
  if (p.crc != tegangCrc8((const uint8_t *)&p, sizeof(p) - 1)) return;
  if (!macAllZero(REMOTE_MAC) && memcmp(info->src_addr, REMOTE_MAC, 6) != 0) return;

  uint32_t now = millis();
  portENTER_CRITICAL(&rxMux);
  bool accept = !rxValid || ((int16_t)(p.seq - rxPkt.seq) > 0) || (now - lastRxMs > 1000);
  if (accept) { rxPkt = p; lastRxMs = now; rxValid = true; }
  portEXIT_CRITICAL(&rxMux);

  if (accept && !peerKnown) {
    memcpy(remoteMac, info->src_addr, 6);
    peerKnown = true;
    needAddPeer = true;
  }
}
Cmd getCmd() {
  Cmd c; memset(&c, 0, sizeof(c));
  portENTER_CRITICAL(&rxMux);
  bool valid = rxValid;
  uint32_t last = lastRxMs;
  RemotePacket p = rxPkt;
  portEXIT_CRITICAL(&rxMux);
  c.link = valid && (millis() - last < LINK_TIMEOUT_MS);
  if (c.link) {
    c.buttons = p.buttons;
    for (int i = 0; i < 4; i++) c.axes[i] = p.axes[i];
  }
  return c;
}
void handlePeerAndStatus() {
  if (needAddPeer && !peerAdded) {
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, remoteMac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) peerAdded = true;
    needAddPeer = false;
  }
  static uint32_t last = 0;
  if (!peerAdded || millis() - last < 200) return;      // status 5 Hz
  last = millis();
  RobotStatusPacket s;
  s.ver = PKT_VER_STATUS;
  s.state = state;
  s.fault = fault;
  s.flags = (battWarn ? FLG_BATT_WARN : 0) | (obsF ? FLG_OBST_FRONT : 0) |
            (obsB ? FLG_OBST_BACK : 0) | (currentMode == MODE_TUNING ? FLG_TUNING : 0);
  s.vbat_cV = vbatValid ? (uint16_t)(vbat * 100.0f) : 0;
  s.pitch_cdeg = (int16_t)(g_pitch * 100.0f);
  s.crc = tegangCrc8((const uint8_t *)&s, sizeof(s) - 1);
  esp_now_send(remoteMac, (const uint8_t *)&s, sizeof(s));
}

// ====================== WEB TUNING (core 0, hanya aktif di MODE_TUNING) ======================
static String statusJson() {
  String j = "{\"state\":\"";
  j += stateName(state);
  j += "\",\"fault\":\"";
  j += faultName(fault);
  j += "\",\"pitch\":";   j += String(g_pitch, 2);
  j += ",\"vbat\":";      j += String(vbatValid ? vbat : 0.0f, 2);
  j += ",\"v\":";         j += String(vAvg, 1);
  j += ",\"link\":";      j += (millis() - lastRxMs < LINK_TIMEOUT_MS && rxValid) ? "true" : "false";
  j += "}";
  return j;
}
static void addField(String &h, const char *label, const char *name, float val, int dec) {
  h += label;
  h += F(" <input name='");
  h += name;
  h += F("' value='");
  h += String(val, dec);
  h += F("'><br>");
}
static String buildPage() {
  String h;
  h.reserve(3500);
  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'><title>Tuning TEGANG</title>"
         "<style>body{font-family:Arial,sans-serif;text-align:center;margin:14px}"
         "input{width:80px;padding:5px;margin:4px;text-align:center;font-size:16px}"
         ".b{background:#007BFF;color:#fff;border:0;border-radius:6px;padding:10px 18px;font-weight:bold;margin:6px;width:auto}"
         "#st{background:#eee;border-radius:6px;padding:8px;margin-bottom:10px;font-size:14px}</style></head><body>"
         "<h2>Tuning PID TEGANG</h2><div id='st'>memuat...</div><form action='/update'>");
  h += F("<h3>Loop sudut</h3>");
  addField(h, "Kp", "kp", P.kp, 2);
  addField(h, "Ki", "ki", P.ki, 2);
  addField(h, "Kd", "kd", P.kd, 2);
  h += F("<h3>Loop kecepatan</h3>");
  addField(h, "Kp", "kpv", P.kpv, 3);
  addField(h, "Ki", "kiv", P.kiv, 3);
  h += F("<h3>Lain-lain</h3>");
  addField(h, "Trim (deg)", "trim", P.trim, 2);
  addField(h, "Duty max (%)", "duty", P.duty * 100.0f, 0);
  addField(h, "V max (cm/s)", "vmax", P.vmax, 1);
  addField(h, "Yaw max", "yaw", P.yawmax, 1);
  h += F("<br><input class='b' type='submit' value='UPDATE'></form>"
         "<form action='/save'><input class='b' type='submit' value='SIMPAN KE FLASH'></form>"
         "<script>function u(){fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('st').innerText="
         "j.state+' | fault: '+j.fault+' | pitch '+j.pitch+' | v '+j.v+' cm/s | batt '+j.vbat+' V | link '+j.link;});}"
         "setInterval(u,700);u();</script></body></html>");
  return h;
}
static void setupWebServer() {
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", buildPage()); });
  server.on("/status", HTTP_GET, []() { server.send(200, "application/json", statusJson()); });
  server.on("/update", HTTP_GET, []() {
    if (server.hasArg("kp"))   P.kp   = clampf(server.arg("kp").toFloat(),   0.0f, 60.0f);
    if (server.hasArg("ki"))   P.ki   = clampf(server.arg("ki").toFloat(),   0.0f, 10.0f);
    if (server.hasArg("kd"))   P.kd   = clampf(server.arg("kd").toFloat(),   0.0f, 5.0f);
    if (server.hasArg("kpv"))  P.kpv  = clampf(server.arg("kpv").toFloat(),  0.0f, 1.0f);
    if (server.hasArg("kiv"))  P.kiv  = clampf(server.arg("kiv").toFloat(),  0.0f, 0.5f);
    if (server.hasArg("trim")) P.trim = clampf(server.arg("trim").toFloat(), -10.0f, 10.0f);
    if (server.hasArg("duty")) P.duty = clampf(server.arg("duty").toFloat() / 100.0f, 0.10f, 0.90f);
    if (server.hasArg("vmax")) P.vmax = clampf(server.arg("vmax").toFloat(), 0.0f, 60.0f);
    if (server.hasArg("yaw"))  P.yawmax = clampf(server.arg("yaw").toFloat(), 0.0f, 100.0f);
    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.on("/save", HTTP_GET, []() {
    saveParams();
    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.begin();
}
void netTask(void *) {
  for (;;) {
    if (currentMode == MODE_TUNING) {
      if (!apStarted) {
        WiFi.softAP("TEGANG_Tuning", "12345678", ESPNOW_CHANNEL, 0, 2);
        setupWebServer();
        apStarted = true;
        Serial.printf("AP aktif: TEGANG_Tuning (pass 12345678) -> http://%s\n", WiFi.softAPIP().toString().c_str());
      }
      server.handleClient();
      vTaskDelay(pdMS_TO_TICKS(5));
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

// ====================== IMU ======================
void calibrateImu() {
  Serial.println("KALIBRASI IMU: TAHAN ROBOT TEGAK dan DIAM selama 3 detik...");
  for (int i = 3; i > 0; i--) {
    Serial.printf("%d...\n", i);
    for (int k = 0; k < 5; k++) { digitalWrite(PIN_LED, HIGH); delay(100); digitalWrite(PIN_LED, LOW); delay(100); }
  }
  mpu.calcOffsets();
  mpu.update();
  Serial.printf("Offset selesai. Sudut awal: %.2f derajat (harus ~0)\n", IMU_ANGLE());
}
void fatalHalt(const char *msg) {
  motorsOff();
  state = RS_FAULT; fault = RF_IMU;
  for (;;) {
    Serial.println(msg);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(150);
  }
}

// ====================== STATE MACHINE & KONTROL ======================
void setFault(uint8_t f) {
  motorsOff();
  state = RS_FAULT;
  fault = f;
  Serial.printf("FAULT: %s\n", faultName(f));
}
void resetControl() {
  iTerm = 0; ivTerm = 0; thetaRefF = 0; vRef = 0; yawCmd = 0;
  uL = 0; uR = 0; satMs = 0; vAvg = 0; vL = 0; vR = 0;
  stallCntL = 0; stallCntR = 0; runCnt = 0;
}
void startArmed() {
  resetControl();
  encPrevL = ENC_L_SIGN * (long)encL.getCount();
  encPrevR = ENC_R_SIGN * (long)encR.getCount();
  lastOuterMs = millis();
  fault = RF_NONE;
  state = RS_ARMED;
  digitalWrite(PIN_STBY, HIGH);
  Serial.println("ARMED");
}
bool canArm(float pitch, float rate, bool linkOk, bool estop) {
  static uint32_t lastMsg = 0;
  const char *why = nullptr;
  if (!linkOk) why = "link remote putus";
  else if (estop) why = "SW2 (e-stop) masih ON";
  else if (fabsf(pitch - P.trim) > ARM_MAX_ANGLE) why = "robot belum tegak (+-3 derajat)";
  else if (fabsf(rate) > ARM_MAX_RATE) why = "robot belum diam";
  else if (battCut || (vbatValid && vbat < 9.8f)) why = "baterai rendah";
  if (why) {
    if (millis() - lastMsg > 1000) { Serial.printf("ARM DITOLAK: %s\n", why); lastMsg = millis(); }
    return false;
  }
  return true;
}

// loop kecepatan 20 Hz: hitung kecepatan roda, cek stall/runaway, hasilkan sudut perintah
void outerStep() {
  const float dtO = OUTER_MS / 1000.0f;
  long cl = ENC_L_SIGN * (long)encL.getCount();
  long cr = ENC_R_SIGN * (long)encR.getCount();
  float cmPerCount = (PI * WHEEL_D_CM) / ENC_CPR_WHEEL;
  vL = (cl - encPrevL) * cmPerCount / dtO;
  vR = (cr - encPrevR) * cmPerCount / dtO;
  encPrevL = cl; encPrevR = cr;
  float vRaw = 0.5f * (vL + vR);
  vAvg += 0.5f * (vRaw - vAvg);
  g_v = vAvg;

  // stall: u besar tapi roda diam
  stallCntL = (fabsf(uL) > STALL_U && fabsf(vL) < STALL_V) ? stallCntL + 1 : 0;
  stallCntR = (fabsf(uR) > STALL_U && fabsf(vR) < STALL_V) ? stallCntR + 1 : 0;
  if (stallCntL >= STALL_WINDOWS || stallCntR >= STALL_WINDOWS) { setFault(RF_STALL); return; }
  // runaway: roda terlalu cepat terlalu lama
  runCnt = (fabsf(vAvg) > RUNAWAY_V) ? runCnt + 1 : 0;
  if (runCnt >= RUNAWAY_WINDOWS) { setFault(RF_RUNAWAY); return; }

  // loop kecepatan -> sudut perintah (+ = condong depan = percepat maju)
  float errV = vRef - vAvg;
  ivTerm = clampf(ivTerm + P.kiv * errV * dtO, -IV_LIMIT, IV_LIMIT);
  float thetaRef = clampf(P.kpv * errV + ivTerm, -THETA_REF_MAX, THETA_REF_MAX);
  thetaRefF += 0.3f * (thetaRef - thetaRefF);
}

void runBalance(float dt, float pitch, float rate, const Cmd &c) {
  // 1. perintah dari remote (mode TUNING: robot diam di tempat)
  float vTarget = 0.0f, yawTarget = 0.0f;
  if (c.link && currentMode == MODE_NORMAL) {
    vTarget = axisToUnit(c.axes[1]) * P.vmax;
    yawTarget = axisToUnit(c.axes[2]) * P.yawmax;
  }
  updateObstacle();
  if (obsF && vTarget > 0.0f) vTarget = 0.0f;       // rem aman: balance tetap jalan
  if (obsB && vTarget < 0.0f) vTarget = 0.0f;
  vRef = slewf(vRef, vTarget, VREF_ACCEL * dt);
  yawCmd = slewf(yawCmd, yawTarget, YAW_SLEW_PER_S * dt);

  // 2. loop kecepatan 20 Hz
  if (millis() - lastOuterMs >= OUTER_MS) {
    lastOuterMs += OUTER_MS;
    outerStep();
    if (state != RS_ARMED) return;
  }

  // 3. loop sudut 100 Hz (D dari gyro)
  float err = pitch - (thetaRefF + P.trim);          // + = condong depan lebih dari target
  iTerm = clampf(iTerm + P.ki * err * dt, -I_LIMIT, I_LIMIT);
  float u = P.kp * err + iTerm + P.kd * rate;
  float scale = vbatValid ? clampf(BATT_NOM_V / vbat, 0.8f, 1.4f) : 1.0f;   // normalisasi duty ke tegangan baterai
  u *= scale;
  float lim = clampf(P.duty, 0.1f, 0.9f) * 255.0f;
  if (fabsf(u) >= 0.95f * lim) satMs += dt * 1000.0f; else satMs = 0.0f;
  if (satMs > SAT_FAULT_MS) { setFault(RF_SATURATED); return; }
  u = clampf(u, -lim, lim);
  g_u = u;

  // 4. campur yaw, slew-rate, kirim ke motor
  float yaw = yawCmd * scale;
  float tL = clampf(u + yaw, -lim, lim);
  float tR = clampf(u - yaw, -lim, lim);
  uL = slewf(uL, tL, SLEW_U_PER_LOOP);
  uR = slewf(uR, tR, SLEW_U_PER_LOOP);
  driveMotors(uL, uR);
}

void controlStep(float dt) {
  mpu.update();
  float pitch = PITCH_SIGN * IMU_ANGLE();
  float rate  = PITCH_SIGN * IMU_RATE();
  g_pitch = pitch; g_rate = rate;

  Cmd c = getCmd();
  static bool armReqHeld = false, prevArmReq = false, armInit = false;
  if (c.link && !armInit) {                                  // link pertama: catat posisi SW1, jangan anggap itu "transisi ON"
    armReqHeld = (c.buttons & BTN_SW1) != 0;
    prevArmReq = armReqHeld;
    armInit = true;
  }
  if (c.link) armReqHeld = (c.buttons & BTN_SW1) != 0;      // link putus: pakai nilai terakhir
  bool armReq = armReqHeld;
  bool estop = c.link && (c.buttons & BTN_SW2);
  bool armEdge = armReq && !prevArmReq;
  prevArmReq = armReq;
  uint32_t linkLost = (rxValid ? (millis() - lastRxMs) : 999999UL);

  switch (state) {
    case RS_DISARMED:
      motorsOff();
      if (armEdge && canArm(pitch, rate, c.link, estop)) startArmed();
      break;

    case RS_ARMED:
      if (estop)                         { setFault(RF_ESTOP);   break; }
      if (!armReq)                       { state = RS_DISARMED; motorsOff(); Serial.println("DISARMED"); break; }
      if (fabsf(pitch) > FALL_DEG)       { setFault(RF_FALL);    break; }
      if (battCut)                       { setFault(RF_LOWBATT); break; }
      if (!c.link && linkLost > LINK_DISARM_MS) { state = RS_DISARMED; motorsOff(); Serial.println("LINK PUTUS -> DISARMED"); break; }
      runBalance(dt, pitch, rate, c);
      break;

    default:   // RS_FAULT (latched): keluar hanya kalau link ada, SW1 dan SW2 OFF
      motorsOff();
      if (c.link && !armReq && !estop) { state = RS_DISARMED; fault = RF_NONE; Serial.println("FAULT dibersihkan -> DISARMED"); }
      break;
  }
}

// ====================== TELEMETRI, LED, SERIAL ======================
void telemetry() {
  static uint32_t last = 0;
  if (!telemetryOn || millis() - last < 50) return;          // 20 Hz
  last = millis();
  Serial.printf("T,%lu,%s,%.2f,%.1f,%.2f,%.0f,%.1f,%.1f,%.2f,%.0f,%.0f\n",
                (unsigned long)millis(), stateName(state), g_pitch, g_rate, thetaRefF, g_u,
                vRef, g_v, vbatValid ? vbat : 0.0f, (float)distF, (float)distB);
}
void updateLed() {
  uint32_t t = millis();
  bool on;
  if (state == RS_FAULT)       on = ((t / 100) % 2) == 0;                     // kedip cepat
  else if (state == RS_ARMED)  on = battWarn ? (((t / 250) % 2) == 0) : true; // menyala (kedip kalau baterai lemah)
  else                         on = (t % 1000) < 100;                         // kedip singkat tiap 1 detik
  digitalWrite(PIN_LED, on);
}
void handleBootButton() {
  static uint32_t pressMs = 0;
  static bool handled = false;
  if (digitalRead(PIN_BOOT) == LOW) {
    if (pressMs == 0) pressMs = millis();
    else if (!handled && millis() - pressMs > 600) {
      currentMode = (currentMode == MODE_NORMAL) ? MODE_TUNING : MODE_NORMAL;
      Serial.println(currentMode == MODE_TUNING ? "MODE: TUNING (remote dinonaktifkan untuk gerak)" : "MODE: NORMAL");
      handled = true;
    }
  } else { pressMs = 0; handled = false; }
}
void benchPulse(bool left) {
  if (state != RS_DISARMED) { Serial.println("Tes motor hanya saat DISARMED."); return; }
  Serial.printf(">> TES RODA %s: ANGKAT ROBOT (roda bebas)! 1 detik, u=+60\n", left ? "KIRI" : "KANAN");
  long c0L = ENC_L_SIGN * (long)encL.getCount();
  long c0R = ENC_R_SIGN * (long)encR.getCount();
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) { driveMotors(left ? 60.0f : 0.0f, left ? 0.0f : 60.0f); delay(5); }
  motorsOff();
  long dL = ENC_L_SIGN * (long)encL.getCount() - c0L;
  long dR = ENC_R_SIGN * (long)encR.getCount() - c0R;
  Serial.printf("Encoder kiri %+ld, kanan %+ld count. Roda harus berputar MAJU dan count POSITIF.\n", dL, dR);
  Serial.println("Salah arah putar -> balik MOTOR_x_SIGN. Count negatif padahal maju -> balik ENC_x_SIGN.");
}
void printHelp() {
  Serial.println("Perintah: t=telemetri on/off  p=lihat parameter  s=simpan parameter  c=kalibrasi IMU");
  Serial.println("          l=tes roda kiri  r=tes roda kanan  h=bantuan   (tahan BOOT 0,6 s = mode tuning web)");
}
void handleSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    switch (ch) {
      case 't': telemetryOn = !telemetryOn; Serial.println(telemetryOn ? "telemetri ON" : "telemetri OFF"); break;
      case 'p': printParams(); break;
      case 's': saveParams(); Serial.println("Parameter disimpan ke flash."); break;
      case 'c': if (state == RS_DISARMED) calibrateImu(); else Serial.println("Disarm dulu."); break;
      case 'l': benchPulse(true); break;
      case 'r': benchPulse(false); break;
      case 'h': printHelp(); break;
      default: break;
    }
  }
}

// ====================== SETUP & LOOP ======================
void setup() {
  // 1) keadaan aman paling awal: driver mati, semua input arah LOW
  pinMode(PIN_STBY, OUTPUT);  digitalWrite(PIN_STBY, LOW);
  pinMode(PIN_IN1_L, OUTPUT); digitalWrite(PIN_IN1_L, LOW);
  pinMode(PIN_IN2_L, OUTPUT); digitalWrite(PIN_IN2_L, LOW);
  pinMode(PIN_IN1_R, OUTPUT); digitalWrite(PIN_IN1_R, LOW);
  pinMode(PIN_IN2_R, OUTPUT); digitalWrite(PIN_IN2_R, LOW);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_TRIG_F, OUTPUT); pinMode(PIN_ECHO_F, INPUT);
  pinMode(PIN_TRIG_B, OUTPUT); pinMode(PIN_ECHO_B, INPUT);
  ledcAttach(PIN_PWM_L, PWM_FREQ, PWM_BITS); ledcWrite(PIN_PWM_L, 0);
  ledcAttach(PIN_PWM_R, PWM_FREQ, PWM_BITS); ledcWrite(PIN_PWM_R, 0);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== TEGANG robot v1.0 ===");
  loadParams();
  printParams();

  // 2) encoder (PCNT, x4)
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encL.attachFullQuad(PIN_ENC_L_A, PIN_ENC_L_B);
  encR.attachFullQuad(PIN_ENC_R_A, PIN_ENC_R_B);
  encL.setFilter(1023); encR.setFilter(1023);
  encL.clearCount();    encR.clearCount();

  // 3) IMU
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);
  byte st = mpu.begin();
  if (st != 0) fatalHalt("IMU MPU6050 TIDAK TERDETEKSI (cek SDA=21, SCL=22, VCC 3V3, GND)");
  mpu.setFilterGyroCoef(0.98f);
  mpu.writeData(0x1A, 0x03);          // DLPF ~42 Hz
  calibrateImu();

  // 4) radio: channel tetap, tanpa power-save, ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.print("MAC robot (isi ke ROBOT_MAC di remote): ");
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) fatalHalt("ESP-NOW GAGAL INISIALISASI");
  esp_now_register_recv_cb(onRecv);

  // 5) task core 0: sensor jarak dan web tuning
  xTaskCreatePinnedToCore(sensorTask, "us",  3072, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(netTask,    "net", 8192, NULL, 1, NULL, 0);

  printHelp();
  Serial.println("Siap. Nyalakan remote, SW1 OFF lalu ON untuk ARM (robot harus tegak dan diam).");
  lastLoopUs = micros();
}

void loop() {
  handleSerial();
  handleBootButton();
  updateLed();
  readBattery();
  handlePeerAndStatus();
  telemetry();

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < LOOP_US) return;
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  if (dt > 0.03f) dt = 0.03f;
  controlStep(dt);
}
