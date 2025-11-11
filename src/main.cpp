#include "somfy.h"

SomfyRtsReceiver somfy;

void setup()
{
  Serial.begin(115200);
  Serial.printf("Somfy RTS receiver v0.1\r\n");

  somfy.init();           // Initialize CC1101 and set frequency to 433.42MHz.s
  somfy.enableReceive(); // configure the receive pin and switch on receive.

  Serial.printf("init completed\r\n");

  delay(100);
  somfy.clear_received();
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
      if (somfy.receive()) {
        Serial.println("Decrypted Code:");
        for (int i = 0; i < 10; i++)
        {
          Serial.print(somfy.received_code(i, 1)); //(addr,1 = decrypted_data) outputs the decrypted data in the serial monitor.
          Serial.print(" ");
        }
        Serial.println("\r\n");
      }
      else
      {
        Serial.printf("Receive failed\r\n");
      }
    }

    somfy.clear_received(); // reset and get ready to receive new ones.
  }

  delay(10);
}