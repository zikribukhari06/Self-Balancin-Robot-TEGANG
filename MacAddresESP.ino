#include <WiFi.h> // library Wifi.h

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.println();
  Serial.print("MAC Address ESP32 ini: "); // example = 7C:E8:B1:B1:E7:B8  for my esp32 s3 , 6C:C8:40:33:C9:A0 for my esp32 dev module jg, 20:50:0D:D0:28:2C for esp32 ipt
  Serial.println(WiFi.macAddress());
}

void loop() {
}
