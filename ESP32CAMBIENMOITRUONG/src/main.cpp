/***************************************************
 * ESP32 + DHT11 + MQ135 + Blynk Cloud
 * Hiển thị: Nhiệt độ (°C), Độ ẩm (%), Chất lượng không khí (MQ135)
 ***************************************************/

#define BLYNK_TEMPLATE_ID "TMPL6DnKK3ryJ"
#define BLYNK_TEMPLATE_NAME "CHAT LUONG PHONG"
#define BLYNK_AUTH_TOKEN "mVxOVMCmpSoEHRfmaMZjrQydImRyjsWC"


#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"

// ====== Thông tin WiFi ======
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Hieu";         
char pass[] = "16012004";     

// ====== Cấu hình cảm biến ======
#define DHTPIN 4           
#define DHTTYPE DHT11
#define MQ135_PIN 34       

DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

// ====== Hàm gửi dữ liệu cảm biến ======
void sendSensorData()
{
  // Đọc dữ liệu cảm biến
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int mq135_value = analogRead(MQ135_PIN);

  // Kiểm tra dữ liệu DHT11
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(" Lỗi đọc dữ liệu DHT11!");
    return;
  }

  // Phân loại chất lượng không khí theo giá trị MQ135
  String airQuality;
  if (mq135_value < 300) airQuality = "Tốt";
  else if (mq135_value < 1000) airQuality = "Trung bình";
  else airQuality = "Ô nhiễm";

  // In ra Serial
  Serial.println("=======================================");
  Serial.print("  Nhiệt độ: "); Serial.print(temperature); Serial.println(" °C");
  Serial.print(" Độ ẩm: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("  MQ135: "); Serial.print(mq135_value);
  Serial.print(" → Chất lượng không khí: "); Serial.println(airQuality);
  Serial.println("=======================================");

  // Gửi dữ liệu lên Blynk
  Blynk.virtualWrite(V1, temperature);      
  Blynk.virtualWrite(V2, humidity);         
  Blynk.virtualWrite(V0, mq135_value);     
  Blynk.virtualWrite(V4, airQuality);       
}

// ====== setup ======
void setup()
{
  Serial.begin(115200);
  dht.begin();

  Serial.println("Kết nối WiFi & Blynk...");
  Blynk.begin(auth, ssid, pass);

  // Cứ 2 giây gửi dữ liệu 1 lần
  timer.setInterval(2000L, sendSensorData);
}

// ====== loop ======
void loop()
{
  Blynk.run();
  timer.run();
}
