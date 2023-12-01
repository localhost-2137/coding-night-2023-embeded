#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFiManager.h> 

#define TRIG_PIN 2
#define ECHO_PIN 4
#define DHT_PIN 15

unsigned long measureDistance();
String getChipID();
void readTemp();

DHT_Unified dht(DHT_PIN, DHT22);
unsigned long dhtDelay;
unsigned long dhtLastRead;

unsigned long lastDst = 1368;
unsigned long dstDelay = 5000;
unsigned long dstLastDelay;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dhtDelay = sensor.min_delay / 1000;
  dhtLastRead = millis();

  WiFiManager wm;

  String ssid = "Smarty-" + getChipID();
  bool res = wm.autoConnect(ssid.c_str());
  if(!res) {
    Serial.println("Failed to connect");
    ESP.restart();
  }

  lastDst = measureDistance();
  dstLastDelay = 5000;
}

void loop() {
  if (millis() - dhtLastRead >= dhtDelay) {
    readTemp();
  }

  unsigned long currDst = measureDistance();
  if (abs(lastDst - currDst * 1.0f) > 10 && millis() - dstLastDelay > dstDelay) {
    dstLastDelay = millis();
    Serial.println("MOVE DETECTED!");
  }

  lastDst = currDst;
  // Serial.println(ESP.getEfuseMac());
  // Serial.println(getChipID());

  delay(50);
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
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
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