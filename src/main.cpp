#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFiManager.h> 
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#define TRIG_PIN 5
#define ECHO_PIN 4
#define DHT_PIN 15
#define LED_PIN LED_BUILTIN

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void moveDetection();
void simulateWattHours();
unsigned long measureDistance();
String getChipID();
void readTemp();

DHT_Unified dht(DHT_PIN, DHT22);
WebSocketsClient webSocket;

float currentTemperature = -1.0f;
float currentHumidity = -1.0f;

unsigned long dhtDelay;
unsigned long dhtLastRead;

unsigned long lastDst = 1368;
unsigned long dstDelay = 5000;
unsigned long dstLastDelay;

unsigned long sendDataDelay = 5000;
unsigned long lastDataSent;

double avgWattHours = 0.289351852;
double currentWattHours = avgWattHours;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dhtDelay = sensor.min_delay / 1000;
  dhtLastRead = millis();

  WiFiManager wm;
  WiFiManagerParameter custom_field;
  String custom_html = "<br /> <h1 style=\"text-align: center;\">Device ID: " + getChipID() + "</h1>";
  new (&custom_field) WiFiManagerParameter(custom_html.c_str());
  
  wm.addParameter(&custom_field);
  // wm.resetSettings();

  String ssid = "Smarty-" + getChipID();
  bool res = wm.autoConnect(ssid.c_str());
  if(!res) {
    Serial.println("Failed to connect");
    ESP.restart();
  }

  webSocket.begin("10.42.0.1", 3000, "/ws");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  webSocket.sendTXT("Hello from ESP32!");

  configTime(3600, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

  lastDst = measureDistance();
  dstLastDelay = millis();
  lastDataSent = millis();
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    // DynamicJsonDocument doc(2048);
    // deserializeJson(doc, payload);

    // Serial.printf("Received message: %s\n", doc["espId"].as<const char *>());
    Serial.printf("Received message: %s\n", payload);
  } else if (type == WStype_CONNECTED) {
    Serial.println("Connected to WebSocket server");
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("Disconnected from WebSocket server");
  }
}

void loop() {
  webSocket.loop();

  if (millis() - dhtLastRead >= dhtDelay) {
    readTemp();
  }
  moveDetection();

  if (millis() - lastDataSent >= sendDataDelay) {
    lastDataSent = millis();

    DynamicJsonDocument doc(256);
  
    doc["device_id"] = ESP.getEfuseMac();
    doc["type"] = "temp";
    doc["data"]["temp"] = currentTemperature;
    doc["data"]["hum"] = currentHumidity;
    doc["data"]["wh"] = currentWattHours;

    String json;
    serializeJson(doc, json);

    webSocket.sendTXT(json);
  }

  delay(50);
}

void simulateWattHours() {

}

void moveDetection() {
  unsigned long currDst = measureDistance();
  if (abs(lastDst - currDst * 1.0f) > 10 && millis() - dstLastDelay > dstDelay) {
    Serial.println("MOVE DETECTED!");
    dstLastDelay = millis();

    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);

    DynamicJsonDocument doc(256);
    doc["device_id"] = ESP.getEfuseMac();
    doc["type"] = "move";

    String json;
    serializeJson(doc, json);

    webSocket.sendTXT(json);
  }
  lastDst = currDst;
}

void readTemp() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));

    currentTemperature = event.temperature;
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));

    currentHumidity = event.relative_humidity;
  }

  dhtLastRead = millis();
}

String getChipID() {
  uint64_t chipid = ESP.getEfuseMac();
  String chipidStr = String((uint32_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);
  return chipidStr;
}

unsigned long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  digitalWrite(ECHO_PIN, HIGH);
  unsigned long time = pulseIn(ECHO_PIN, HIGH);
  unsigned long dst = time / 58;

  return dst;
}