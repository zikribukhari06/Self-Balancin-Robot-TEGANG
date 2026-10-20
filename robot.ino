#include <Wire.h>
#include <MPU6050_light.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>

// --- KONFIGURASI PIN ROBOT ---
#define BTN_BOOT 0
#define PWM_L 13
#define IN1_L 4
#define IN2_L 14
#define PWM_R 25
#define IN1_R 26
#define IN2_R 27
#define STBY_PIN 15
#define TRIG_F 5
#define ECHO_F 18
#define TRIG_B 19
#define ECHO_B 23
#define ENC_L_A 16
#define ENC_L_B 17
#define ENC_R_A 32
#define ENC_R_B 33

// --- VARIABEL GLOBAL ---
MPU6050 mpu(Wire);
WebServer server(80);
TaskHandle_t NetworkTask;

enum SystemMode { MODE_ESPNOW, MODE_TUNING };
volatile SystemMode currentMode = MODE_ESPNOW;

// Struktur Data ESP-NOW
typedef struct struct_message {
  float pitch_cmd;  
  float yaw_cmd;    
  bool sw1, sw2, sw3, sw4;
} struct_message;
struct_message remoteData;

// Variabel Waktu & Kontrol Keseimbangan (Inner PID)
unsigned long prevTime = 0;
const unsigned long loopTime = 10000; // 100Hz = 10ms
float dt = 0.01;
float Kp_in = 15.0, Ki_in = 1.0, Kd_in = 0.5; 
float target_pitch = 0; 
float sigma_error_in = 0, last_error_in = 0;

// Variabel Kontrol Posisi (Outer PID)
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;
float Kp_out = 2.0, Ki_out = 0.1, Kd_out = 0.1;
float sigma_error_out = 0, last_error_out = 0;
float pitch_offset = 0; 

// --- TEMPLATE HTML WEBSERVER ---
const char* html_page = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Tuning TEGANG</title>
<style>
  body { font-family: Arial, sans-serif; text-align: center; margin-top: 30px; }
  input { width: 80px; padding: 5px; margin: 5px; text-align: center; font-size: 16px; }
  input[type=submit] { background-color: #007BFF; color: white; border: none; border-radius: 5px; padding: 10px 20px; font-weight: bold; }
</style></head><body>
  <h2>Tuning PID TEGANG</h2>
  <form action="/update">
    <h3>Inner Loop (Sudut)</h3>
    Kp: <input type="text" name="kpi" value="%.2f"><br>
    Ki: <input type="text" name="kii" value="%.2f"><br>
    Kd: <input type="text" name="kdi" value="%.2f"><br>
    <h3>Outer Loop (Posisi)</h3>
    Kp: <input type="text" name="kpo" value="%.2f"><br>
    Ki: <input type="text" name="kio" value="%.2f"><br>
    Kd: <input type="text" name="kdo" value="%.2f"><br><br>
    <input type="submit" value="UPDATE PID">
  </form>
</body></html>
)rawliteral";

// --- ISR ENCODER & TOMBOL ---
void IRAM_ATTR toggleMode() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 300) { 
    currentMode = (currentMode == MODE_ESPNOW) ? MODE_TUNING : MODE_ESPNOW;
  }
  last_interrupt_time = interrupt_time;
}
void IRAM_ATTR isrEncoderL() { digitalRead(ENC_L_B) == HIGH ? encoderCountL++ : encoderCountL--; }
void IRAM_ATTR isrEncoderR() { digitalRead(ENC_R_B) == HIGH ? encoderCountR++ : encoderCountR--; }

// --- CALLBACK ESP-NOW ---
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&remoteData, incomingData, sizeof(remoteData));
}

// --- FUNGSI WEBSERVER & JARINGAN (CORE 0) ---
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), html_page, Kp_in, Ki_in, Kd_in, Kp_out, Ki_out, Kd_out);
    server.send(200, "text/html", buffer);
  });
  server.on("/update", HTTP_GET, []() {
    if (server.hasArg("kpi")) Kp_in = server.arg("kpi").toFloat();
    if (server.hasArg("kii")) Ki_in = server.arg("kii").toFloat();
    if (server.hasArg("kdi")) Kd_in = server.arg("kdi").toFloat();
    if (server.hasArg("kpo")) Kp_out = server.arg("kpo").toFloat();
    if (server.hasArg("kio")) Ki_out = server.arg("kio").toFloat();
    if (server.hasArg("kdo")) Kd_out = server.arg("kdo").toFloat();
    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.begin();
}

void networkTaskCode( void * pvParameters ) {
  for(;;) {
    if(currentMode == MODE_TUNING) server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// --- FUNGSI HARDWARE (SENSOR & MOTOR) ---
float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000); 
  return (duration == 0) ? 100.0 : (duration * 0.034 / 2);
}

void driveMotor(float pwm_L, float pwm_R) {
  int outL = constrain(abs(pwm_L), 0, 255);
  int outR = constrain(abs(pwm_R), 0, 255);
  
  digitalWrite(IN1_L, pwm_L >= 0 ? HIGH : LOW);
  digitalWrite(IN2_L, pwm_L >= 0 ? LOW : HIGH);
  analogWrite(PWM_L, outL);
  
  digitalWrite(IN1_R, pwm_R >= 0 ? HIGH : LOW);
  digitalWrite(IN2_R, pwm_R >= 0 ? LOW : HIGH);
  analogWrite(PWM_R, outR);
}

void computeOuterPID() {
  noInterrupts();
  long countL = encoderCountL; long countR = encoderCountR;
  encoderCountL = 0; encoderCountR = 0;
  interrupts();

  float actual_speed = (countL + countR) / 2.0;
  float target_speed = remoteData.pitch_cmd * 15.0; // Kalibrasi faktor pengali ini nanti
  float error_out = target_speed - actual_speed;
  
  float P_out = Kp_out * error_out;
  sigma_error_out = constrain(sigma_error_out + error_out, -50.0, 50.0);
  float I_out = Ki_out * sigma_error_out;
  float D_out = Kd_out * (error_out - last_error_out);
  last_error_out = error_out;

  pitch_offset = constrain(P_out + I_out + D_out, -10.0, 10.0);
}

// --- SETUP & MAIN LOOP (CORE 1) ---
void setup() {
  Serial.begin(115200);
  
  // Pin Setup
  pinMode(BTN_BOOT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_BOOT), toggleMode, FALLING);
  pinMode(PWM_L, OUTPUT); pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(PWM_R, OUTPUT); pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncoderR, RISING);

  // MPU & Jaringan
  Wire.begin(21, 22); mpu.begin(); mpu.calcOffsets(true, true);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("TEGANG_Tuning", "12345678");
  setupWebServer();
  
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  xTaskCreatePinnedToCore(networkTaskCode, "NetTask", 10000, NULL, 1, &NetworkTask, 0);
  
  prevTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  static unsigned long lastOuterTime = 0;
  static int us_counter = 0;
  static float dist_F = 100.0, dist_B = 100.0;
  
  // Outer Loop 20Hz (Tiap 50ms)
  if (currentTime - lastOuterTime >= 50000) { 
    lastOuterTime = currentTime;
    computeOuterPID();
  }

  // Inner Loop 100Hz (Tiap 10ms)
  if (currentTime - prevTime >= loopTime) {
    dt = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;

    mpu.update();
    float actual_pitch = mpu.getAngleY(); // Ubah ke getAngleX() jika orientasi sensor kebalik
    
    // Baca Ultrasonik 10Hz
    if (us_counter++ >= 10) {
      dist_F = getDistance(TRIG_F, ECHO_F);
      dist_B = getDistance(TRIG_B, ECHO_B);
      us_counter = 0;
    }

    // Navigasi & Safety Override
    if (dist_F <= 15.0 && remoteData.pitch_cmd > 0) target_pitch = 0;
    else if (dist_B <= 15.0 && remoteData.pitch_cmd < 0) target_pitch = 0;
    else target_pitch = (currentMode == MODE_ESPNOW) ? pitch_offset : 0; 

    // Kalkulasi Inner PID
    float error_in = target_pitch - actual_pitch;
    float P_in = Kp_in * error_in;
    sigma_error_in = constrain(sigma_error_in + (error_in * dt), -255, 255);
    float I_in = Ki_in * sigma_error_in;
    float D_in = Kd_in * ((error_in - last_error_in) / dt);
    last_error_in = error_in;

    float PID_value = P_in + I_in + D_in;
    float pwm_L = PID_value + remoteData.yaw_cmd;
    float pwm_R = PID_value - remoteData.yaw_cmd;

    // Cut-off jatuh
    if(abs(actual_pitch) > 45) { pwm_L = 0; pwm_R = 0; sigma_error_in = 0; }
    
    driveMotor(pwm_L, pwm_R);
  }
}
