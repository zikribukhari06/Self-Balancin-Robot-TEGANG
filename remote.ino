#include <WiFi.h>
#include <esp_now.h>

// --- GANTI DENGAN MAC ADDRESS ESP32 ROBOTMU ---
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

// --- KONFIGURASI PIN REMOTE ---
#define JOY_PITCH 34 // Joystick Kiri Y (Maju Mundur)
#define JOY_YAW   35 // Joystick Kanan X (Belok Kanan Kiri)
#define SW1 25
#define SW2 26
#define SW3 27
#define SW4 14

// Struktur Data harus sama persis dengan Robot
typedef struct struct_message {
  float pitch_cmd;  
  float yaw_cmd;    
  bool sw1, sw2, sw3, sw4;
} struct_message;
struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Status Kirim: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sukses" : "Gagal");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inisialisasi ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Gagal menambah peer");
    return;
  }
}

void loop() {
  int rawPitch = analogRead(JOY_PITCH); // Nilai 0 - 4095
  int rawYaw = analogRead(JOY_YAW);     // Nilai 0 - 4095

  // Mapping dengan Deadband (agar stabil di tengah saat tidak disentuh)
  // Pitch target: Maju (+5 derajat) ke Mundur (-5 derajat)
  if (rawPitch < 1800) {
    myData.pitch_cmd = map(rawPitch, 0, 1800, 500, 0) / 100.0;
  } else if (rawPitch > 2300) {
    myData.pitch_cmd = map(rawPitch, 2300, 4095, 0, -500) / 100.0;
  } else {
    myData.pitch_cmd = 0.0;
  }

  // Yaw target: Belok dengan membedakan PWM Kiri dan Kanan (Max +- 50)
  if (rawYaw < 1800) {
    myData.yaw_cmd = map(rawYaw, 0, 1800, -50, 0);
  } else if (rawYaw > 2300) {
    myData.yaw_cmd = map(rawYaw, 2300, 4095, 0, 50);
  } else {
    myData.yaw_cmd = 0.0;
  }

  // Baca tombol (karena pullup, ditekan = LOW, maka kita negasikan agar terbaca True)
  myData.sw1 = !digitalRead(SW1);
  myData.sw2 = !digitalRead(SW2);
  myData.sw3 = !digitalRead(SW3);
  myData.sw4 = !digitalRead(SW4);

  // Kirim data via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  delay(20); // Loop 50Hz, tidak perlu pakai micros() karena ini cuma pengirim
}
