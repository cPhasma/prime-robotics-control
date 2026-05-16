#include <ESP8266WiFi.h>
#include <WiFiClient.h>

const char* ssid = "S25_FE_user";
const char* password = "nmcw44ecvdcizs5";

const char* serverHost = "10.165.187.4";
const uint16_t serverPort = 5001;

// Для первой машинки оставь "car1".
// Для второй машинки перед прошивкой поменяй на "car2".
const char* carId = "car1";

WiFiClient client;
unsigned long lastReconnect = 0;
unsigned long lastHeartbeat = 0;
const unsigned long RECONNECT_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 10000;

String serialBuffer = "";
String tcpBuffer = "";

void requestCapabilitiesFromArduino();

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);

  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW);
  }

  connectToServer();
}

void loop() {
  if (!client.connected()) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (millis() - lastReconnect > RECONNECT_INTERVAL) {
      connectToServer();
      lastReconnect = millis();
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);

    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
      client.print("{\"type\":\"heartbeat\",\"car_id\":\"");
      client.print(carId);
      client.println("\"}");
      lastHeartbeat = millis();
    }

    while (client.available() > 0) {
      char c = (char)client.read();
      if (c == '\n') {
        tcpBuffer.trim();
        if (tcpBuffer.length() > 0) {
          Serial.println(tcpBuffer);
        }
        tcpBuffer = "";
      } else if (c != '\r') {
        tcpBuffer += c;
      }
    }
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0 && client.connected()) {
        client.println(serialBuffer);
      }
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }

  delay(1);
}

void connectToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (client.connected()) {
    client.stop();
  }

  if (client.connect(serverHost, serverPort)) {
    lastHeartbeat = millis();
    client.print("{\"type\":\"hello\",\"device\":\"esp8266\",\"car_id\":\"");
    client.print(carId);
    client.println("\"}");

    // После подключения сразу просим Arduino повторно отправить список команд.
    // Это исправляет ситуацию, когда Arduino отправила capabilities при старте,
    // но ESP ещё не была подключена к серверу.
    delay(150);
    requestCapabilitiesFromArduino();
  }
}

void requestCapabilitiesFromArduino() {
  Serial.println("{\"cmd\":\"get_capabilities\",\"command_id\":\"esp_startup_capabilities\"}");
}
