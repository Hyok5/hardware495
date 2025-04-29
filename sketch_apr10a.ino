#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Preferences.h>
#include <math.h>

// ====== CONFIGURATION ======
const char* ssid = "Thitiporn";
const char* password = "123456781";
const char* mqttServer = "broker.emqx.io";  
const int mqttPort = 1883;
const char* mqttTopicPub = "esp32/test/mqttx";
const char* mqttTopicSub = "esp32/test/mqttx";

// ===== DHT Sensor =====
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ===== MQ2 / MQ135 =====
#define MQ2_PIN 35
#define MQ135_PIN 34
#define RL_MQ2 5.0
#define RL_MQ135 10.0
float R0_MQ2 = 10.0;
float R0_MQ135 = 10.0;

// ===== PMS7003 UART =====
HardwareSerial pmsSerial(2);
#define PMS_RX 16
#define PMS_TX 17

WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

// ====== Utility ======
float getResistance(int adc, float RL) {
  float voltage = adc * (3.3 / 4095.0);
  if (voltage == 0) return -1;
  return (3.3 - voltage) * RL / voltage;
}

// ปรับสูตร PPM ให้ตอบสนองเร็วขึ้น
float getMQ2PPM(float rs) {
  float ratio = rs / R0_MQ2;
  return pow(10, (-0.65 * log10(ratio) + 1.0));  // สูตรใหม่ที่ตอบสนองไวขึ้น
}

float getMQ135PPM(float rs) {
  float ratio = rs / R0_MQ135;
  return pow(10, (-0.55 * log10(ratio) + 1.2));  // สูตรใหม่ที่ตอบสนองไวขึ้น
}

// แยกการคำนวณ SO2 และ NO2
float getSO2PPM(float rs) {
  float ratio = rs / R0_MQ135;
  return pow(10, (-0.38 * log10(ratio) + 0.5));  // สูตรสำหรับ SO2
}

float getNO2PPM(float rs) {
  float ratio = rs / R0_MQ135;
  return pow(10, (-0.42 * log10(ratio) + 0.6));  // สูตรสำหรับ NO2
}

bool readPMS7003(uint16_t &pm1, uint16_t &pm2_5, uint16_t &pm10) {
  if (pmsSerial.available() < 32) return false;
  if (pmsSerial.read() != 0x42 || pmsSerial.read() != 0x4D) {
    while (pmsSerial.available()) pmsSerial.read();  // flush
    return false;
  }

  uint8_t buffer[30];
  pmsSerial.readBytes(buffer, 30);

  uint16_t sum = 0x42 + 0x4D;
  for (int i = 0; i < 28; i++) sum += buffer[i];
  uint16_t checksum = buffer[28] << 8 | buffer[29];
  if (sum != checksum) return false;

  pm1 = buffer[4] << 8 | buffer[5];
  pm2_5 = buffer[6] << 8 | buffer[7];
  pm10 = buffer[8] << 8 | buffer[9];

  return true;
}

// ====== Calibration ======
void calibrateSensors() {
  float sum_mq2 = 0;
  float sum_mq135 = 0;
  const int samples = 100;  // จำนวนการอ่านค่า

  for (int i = 0; i < samples; i++) {
    sum_mq2 += getResistance(analogRead(MQ2_PIN), RL_MQ2);   // อ่านค่า MQ2
    sum_mq135 += getResistance(analogRead(MQ135_PIN), RL_MQ135); // อ่านค่า MQ135
    delay(50);  // หยุดเล็กน้อยเพื่อความแม่นยำ
  }

  R0_MQ2 = sum_mq2 / samples;  // คำนวณค่า R0 สำหรับ MQ2
  R0_MQ135 = sum_mq135 / samples;  // คำนวณค่า R0 สำหรับ MQ135

  preferences.putFloat("R0_MQ2", R0_MQ2);  // เก็บค่า R0 สำหรับ MQ2
  preferences.putFloat("R0_MQ135", R0_MQ135);  // เก็บค่า R0 สำหรับ MQ135

  Serial.printf("✅ คาลิเบรตสำเร็จ R0_MQ2 = %.2f, R0_MQ135 = %.2f\n", R0_MQ2, R0_MQ135);
}

// ====== Calibration ======
void autoCalibrateIfNeeded() {
  preferences.begin("mq_calib", false);
  if (!preferences.isKey("R0_MQ2") || !preferences.isKey("R0_MQ135")) {
    Serial.println("🧪 กำลังคาลิเบรต MQ2 / MQ135...");
    float sum_mq2 = 0;
    float sum_mq135 = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
      sum_mq2 += getResistance(analogRead(MQ2_PIN), RL_MQ2);
      sum_mq135 += getResistance(analogRead(MQ135_PIN), RL_MQ135);
      delay(50);
    }

    R0_MQ2 = sum_mq2 / samples;
    R0_MQ135 = sum_mq135 / samples;
    preferences.putFloat("R0_MQ2", R0_MQ2);
    preferences.putFloat("R0_MQ135", R0_MQ135);
    Serial.printf("✅ คาลิเบรต R0_MQ2 = %.2f, R0_MQ135 = %.2f\n", R0_MQ2, R0_MQ135);
  } else {
    R0_MQ2 = preferences.getFloat("R0_MQ2");
    R0_MQ135 = preferences.getFloat("R0_MQ135");
    Serial.printf("📥 โหลดค่า R0: MQ2 = %.2f, MQ135 = %.2f\n", R0_MQ2, R0_MQ135);
  }
  preferences.end();
}

// ====== MQTT ======
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("📥 MQTTX ส่งมาว่า:");
  Serial.print("🧭 Topic: ");
  Serial.println(topic);

  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("💬 Message: ");
  Serial.println(msg);
}
void reconnectMQTT() {
  if (!client.connected()) {
    Serial.print("🔌 MQTTX กำลังเชื่อมต่อ...");
    String clientId = "ESP32-" + String(random(0xffff), HEX);
    client.setKeepAlive(60);
    
    int retryCount = 0;
    int backoffTime = 5000;  // เริ่มต้นที่ 5 วินาที

    while (!client.connected() && retryCount < 5) {  // Retry 5 ครั้งก่อนหยุด
      if (client.connect(clientId.c_str())) {
        Serial.println(" ✅ เชื่อมต่อแล้ว!");
        client.subscribe(mqttTopicSub);
      } else {
        Serial.print(" ❌ ล้มเหลว rc=");
        Serial.println(client.state());
        delay(backoffTime);  // หน่วงเวลาแบบ Exponential Backoff
        backoffTime *= 2;  // เพิ่มเวลาให้ยาวขึ้นทุกครั้ง
        retryCount++;
      }
    }
    
    // ถ้าไม่สามารถเชื่อมต่อได้หลังจากพยายาม 5 ครั้ง
    if (!client.connected()) {
      Serial.println("⚠️ ไม่สามารถเชื่อมต่อ MQTT ได้ หลังจากพยายาม 5 ครั้ง");
    }
  }
}

void publishData(String data) {
  if (client.connected()) {
    Serial.println("📤 Publishing to MQTT...");
    client.publish(mqttTopicPub, data.c_str());
    Serial.println("✅ Payload sent successfully");
  } else {
    Serial.println("⚠️ MQTT not connected");
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  dht.begin();
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);

  // เรียกใช้งานการคาลิเบรต
  calibrateSensors();  // เรียกฟังก์ชันคาลิเบรต MQ2 และ MQ135
  autoCalibrateIfNeeded();  // อ่านค่า R0 จาก Preferences ถ้ามี

  WiFi.begin(ssid, password);
  Serial.println("📶 กำลังเชื่อมต่อ WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\n✅ WiFi IP: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}
void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("🔴 WiFi หายไป กำลังเชื่อมต่อใหม่...");
    WiFi.disconnect();
    WiFi.reconnect();
  }
}

// ====== Loop ======
void loop() {
  checkWiFi();  // ตรวจสอบสถานะการเชื่อมต่อ WiFi
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("⚠️ DHT อ่านค่าไม่ได้");
    delay(5000);
    return;
  }

  // MQ2
  int mq2ADC = analogRead(MQ2_PIN);
  float rs_mq2 = getResistance(mq2ADC, RL_MQ2);
  float co_ppm = getMQ2PPM(rs_mq2);

  // MQ135
  int mq135ADC = analogRead(MQ135_PIN);
  float rs_mq135 = getResistance(mq135ADC, RL_MQ135);
  float so2_ppm = getSO2PPM(rs_mq135);  // ค่า SO2
  float no2_ppm = getNO2PPM(rs_mq135);  // ค่า NO2

  // PMS7003
  uint16_t pm1 = 0, pm2_5 = 0, pm10 = 0;
  if (!readPMS7003(pm1, pm2_5, pm10)) {
    Serial.println("⚠️ อ่านค่า PM ไม่สำเร็จ");
  }

  // Build JSON Payload
  String payload = "{";
  payload += "\"temp\":" + String(temperature, 1);
  payload += ",\"humidity\":" + String(humidity, 1);
  payload += ",\"co_ppm\":" + String(co_ppm, 2);
  payload += ",\"so2_ppm\":" + String(so2_ppm, 3);  // ค่า SO2 แยก
  payload += ",\"no2_ppm\":" + String(no2_ppm, 3);  // ค่า NO2 แยก
  payload += ",\"pm1\":" + String(pm1);
  payload += ",\"pm2_5\":" + String(pm2_5);
  payload += ",\"pm10\":" + String(pm10);
  payload += "}";

  Serial.println("--------------------------------------------------");
  Serial.println("📦 Payload:");
  Serial.println(payload);

  publishData(payload);
  delay(60000);  // ตั้งเวลาให้ส่งข้อมูลทุก ๆ 20 วินาที
}