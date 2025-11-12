#include <ESP8266WiFi.h>

#include "somfy.h"

static SomfyRtsReceiver somfy;

constexpr unsigned RELAY_PIN = 2; // GPIO2 (D4 on Wemos D1 mini)

void setup()
{
  WiFi.mode(WIFI_STA); // Or WIFI_AP or WIFI_OFF
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF); // Disables all WiFi functionality

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  Serial.begin(115200);
  Serial.printf("Somfy RTS receiver v0.1\r\n");

  somfy.init();           // Initialize CC1101 and set frequency to 433.42MHz
  somfy.enableReceive();  // configure the receive pin and switch on receive.

  Serial.printf("init completed\r\n");

  delay(100);
  somfy.clear();
  Serial.println("listening..\r\n");
}

void loop()
{
  auto state = somfy.getState();

  if ((state == ReceiverState::FINISHED) ||
      (state == ReceiverState::DATA_ERROR) ||
      (state == ReceiverState::PREAMBLE_ERROR))
  {
    if (state != ReceiverState::FINISHED) {
      somfy.printDebug();
    } else {
      if (!somfy.receive()) {
        Serial.printf("Receive failed\r\n");
      }

      Serial.println("Decrypted Code:");

      Message msg = somfy.getMessage();
      Serial.printf("remote ID: 0x%02x%02x\r\n", msg.frame.remoteId0, msg.frame.remoteId1);
      Serial.printf("button ID: 0x%02x\r\n", msg.frame.buttonId);
      Serial.printf("counter: 0x%02x%02x\r\n", msg.frame.counter[0], msg.frame.counter[1]);
      Serial.printf("const: 0x%02x%02x%02x\r\n", msg.frame.const_data[0], msg.frame.const_data[1], msg.frame.const_data[2]);
      Serial.println("--------");

      // try matching anyway, it looks like there is a difference between 'super short' button press and normal button press
      if ((msg.frame.remoteId0 == 0x9c) && (msg.frame.remoteId1 == 0x32) && (msg.frame.buttonId == 0xE0)) {
        Serial.printf("Trigger relay\r\n");
        digitalWrite(RELAY_PIN, LOW);
        delay(100);
        digitalWrite(RELAY_PIN, HIGH);
      }
    }

    somfy.clear(); // reset and get ready to receive new ones.
  }

  delay(100);
}