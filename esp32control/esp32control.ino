#define BLYNK_TEMPLATE_ID "TMPL6vChDb8KF"
#define BLYNK_TEMPLATE_NAME "CUA TU DONG"
#define BLYNK_AUTH_TOKEN "2pWqFWC3yY0PPvquDQWPLCIYs5cYLFt_"


#include <WiFi.h>
#include <WiFiManager.h>
#include <esp_now.h>
#include <BlynkSimpleEsp32.h>

// ---------- Cấu hình chân ----------
const int PIR_SENSOR_OUTPUT_PIN = 18;
const int ESPCAM_CONTROL_PIN = 17;
const int DOOR_CONTROL_PIN = 16;

// ---------- ESP-NOW ----------
uint8_t ESPCAM_mac[] = {0xd8, 0x13, 0x2a, 0x7c, 0x4c, 0xc4}; // MAC ESP32 nhận
uint8_t ESP32_mac[] = {0x80, 0xF3, 0xDA, 0x5E, 0xFC, 0x94};// 

typedef struct  {
  bool modeWEB;
} struct_message;

typedef struct  {
  bool open_door;
} open_message;

open_message door_signal;  // Renamed to avoid confusion with 'open' keyword-like name
struct_message myData;
esp_now_peer_info_t peerInfo;

// Retry variables
bool retryNeeded = false;
bool lastMode = false;  // Lưu mode cuối để retry
unsigned long lastSendTime = 0;
const unsigned long RETRY_INTERVAL = 3000;  // 3 giây

// Door control variables
bool doorOpening = false;
unsigned long doorOpenStartTime = 0;
const unsigned long DOOR_OPEN_DURATION = 5000;  // 5 giây

// ---------- Blynk ----------
char auth[] = BLYNK_AUTH_TOKEN;
bool modeCAM = false;  // trạng thái từ Switch

// ---------- WiFiManager ----------
WiFiManager wifiManager;

// ---------- PIR/LED ----------
bool ledState = false;
unsigned long ledStartTime = 0;

// ---------- Callback ESP-NOW ----------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success");
    retryNeeded = false;  // Thành công -> reset retry
    
  } else {
    Serial.println("Delivery Fail");
    retryNeeded = true;   // Fail -> cần retry
    lastSendTime = millis();
  }
}

// ESP-NOW Receive callback
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.print("Received from: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (len == sizeof(open_message)) {
    open_message received;
    memcpy(&received, incomingData, sizeof(received));
    door_signal.open_door = received.open_door;  // Fixed: use correct field name
    Serial.printf("Received door signal: %s\n", door_signal.open_door ? "OPEN" : "CLOSE");  
    
    if (door_signal.open_door && !doorOpening) {
      // Mở cửa khi nhận tín hiệu open
      Serial.println("Opening door from ESP-NOW signal");
      digitalWrite(DOOR_CONTROL_PIN, HIGH);
      doorOpening = true;
      doorOpenStartTime = millis();
    }
  }
}

// ---------- Callback WiFiManager ----------
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("ESP32 vào chế độ cấu hình WiFi");
}

void saveConfigCallback() {
  Serial.println("WiFi đã được lưu");
}

// ---------- Callback Blynk Switch ----------
BLYNK_WRITE(V1) {
  int pinValue = param.asInt();
  modeCAM = (pinValue == 1);
  Serial.println("Switch thay đổi: Mode " + String(modeCAM ? "ON" : "OFF"));

  // Reset retry nếu có lệnh mới
  retryNeeded = false;

  // Gửi dữ liệu ESP-NOW ngay
  myData.modeWEB = modeCAM;
  lastMode = modeCAM;
  esp_err_t result = esp_now_send(ESPCAM_mac, (uint8_t *)&myData, sizeof(myData));
  if(result == ESP_OK){
    Serial.println("ESP-NOW sent successfully (lần đầu)");
  } else {
    Serial.println("ESP-NOW send error (lần đầu)");
  }
}

BLYNK_WRITE(V0) {
  int pinValue = param.asInt();
  if (pinValue == 1) {  // Chỉ hành động khi ON
    Serial.println("Mở khóa từ Blynk V0");
    if (!doorOpening) {
      digitalWrite(DOOR_CONTROL_PIN, HIGH);
      doorOpening = true;
      doorOpenStartTime = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);

  // ---------- Khởi tạo WiFiManager ----------
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  if (!wifiManager.autoConnect("ESP32-Config")) {
    Serial.println("Không thể kết nối WiFi, ESP sẽ KO reset");
    delay(3000);
    ESP.restart();
  }
  Serial.println("Đã kết nối WiFi: " + WiFi.SSID());

  // ---------- Khởi tạo Blynk ----------
  Blynk.config(auth);  // Cấu hình Blynk với auth token (sau khi WiFi đã kết nối)
  Blynk.connect();     // Kết nối đến Blynk server (sử dụng WiFi đã có từ WiFiManager)

  // ---------- Khởi tạo ESP-NOW ----------
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, ESPCAM_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);  // Đăng ký callback nhận
  Serial.println("ESP-NOW Receiver ready. MAC: " + WiFi.macAddress());
  


  Serial.println("Setup hoàn tất");

  // ---------- Khởi tạo chân ----------
  pinMode(PIR_SENSOR_OUTPUT_PIN, INPUT);
  pinMode(ESPCAM_CONTROL_PIN, OUTPUT);
  digitalWrite(ESPCAM_CONTROL_PIN, LOW);
  pinMode(DOOR_CONTROL_PIN, OUTPUT);
  digitalWrite(DOOR_CONTROL_PIN, LOW);

  Serial.println("Dang lam nong cam bien, xin cho 15s");
  delay(15000);
  Serial.println("Cam bien san sang");
}

void loop() {
  Blynk.run();

  unsigned long currentMillis = millis();

  // ---------- Xử lý đóng cửa tự động ----------
  if (doorOpening && (currentMillis - doorOpenStartTime >= DOOR_OPEN_DURATION)) {
    digitalWrite(DOOR_CONTROL_PIN, LOW);
    doorOpening = false;
    Serial.println("Door closed automatically");
  }

  // ---------- Đọc PIR ----------
  if (digitalRead(PIR_SENSOR_OUTPUT_PIN) == HIGH && !ledState) {
    digitalWrite(ESPCAM_CONTROL_PIN, HIGH);  // Bật LED/relay
    ledStartTime = currentMillis;          // lưu thời điểm bật
    ledState = true;
  }

  // ---------- Tắt LED/relay sau 20s ----------
  if (ledState && !modeCAM && (currentMillis - ledStartTime >= 20000)) {
    digitalWrite(ESPCAM_CONTROL_PIN, LOW);
    ledState = false;
  }

  // ---------- Retry ESP-NOW ----------
  if (retryNeeded && (currentMillis - lastSendTime >= RETRY_INTERVAL)) {
    Serial.println("Retry gửi lệnh: Mode " + String(lastMode ? "ON" : "OFF"));
    myData.modeWEB = lastMode;
    esp_err_t result = esp_now_send(ESPCAM_mac, (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("ESP-NOW retry sent successfully");
      lastSendTime = currentMillis;
    } else {
      Serial.println("ESP-NOW retry send error");
      lastSendTime = currentMillis;
    }
  }
}