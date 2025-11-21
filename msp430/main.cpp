s#include <msp430.h>

#include "drivers/clock/clock.h"
#include "drivers/somfy/somfy.h"
#include "drivers/wdt/wdt.h"

static void loop();

SomfyRtsReceiver somfy;

int main()
{
    Wdt().disable();

    // set P3 for power saving
    P3DIR = 0xFF;
    P3OUT = 0;

    P2DIR |= BIT1;s
    P2OUT &= ~BIT1;

    initFastClock();

    SomfyRtsReceiver somfy;
    somfy.init();
    somfy.enableReceive();

    while(1) {
        loop();
    }
}

static void delay(uint32_t delay)
{
    while(delay--) {
        asm volatile(" NOP");
    }
}

static void toggleRelay()
{
    P2OUT |= BIT1;
    delay(0xFF);
    P2OUT &= ~BIT1;
}

static void loop()
{
    auto state = somfy.getState();

    if ((state == ReceiverState::FINISHED) ||
        (state == ReceiverState::DATA_ERROR) ||
        (state == ReceiverState::PREAMBLE_ERROR))
    {
      if (state != ReceiverState::FINISHED) {
        somfy.printDebug();
      } else {
        somfy.receive();
        Message msg = somfy.getMessage();
        // try matching anyway, it looks like there is a difference between 'super short' button press and normal button press
        if ((msg.frame.remoteId0 == 0x9c) && (msg.frame.remoteId1 == 0x32) && (msg.frame.buttonId == 0xE0)) {
          toggleRelay();
        }
      }

      somfy.clear(); // reset and get ready to receive new ones.
    }

    LPM1;
    delay(0xFFF);
}
