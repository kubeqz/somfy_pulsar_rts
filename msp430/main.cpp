#include <msp430.h>

#include "drivers/clock/clock.h"
#include "drivers/somfy/somfy.h"
#include "drivers/wdt/wdt.h"

class SomfyRtsListenerImpl : public SomfyRtsListener
{
public:
    SomfyRtsListenerImpl(SomfyRtsReceiver& somfy)
        : m_somfy(somfy)
    {
        initRelay();
        m_somfy.setListener(*this);
    }

    void delayMs(unsigned ms)
    {
        while (--ms) {
            // assuming 8MHz clock
            __delay_cycles(80);
        }
    }

    void onReceiveFinished() final
    {
        // we're in interrupt context here

        // ignore result
        m_somfy.receive();

        Message msg = m_somfy.getMessage();
        // try matching anyway, it looks like there is a difference between 'super short' button press and normal button press
        if ((msg.frame.remoteId0 == 0x9c) && (msg.frame.remoteId1 == 0x32) && (msg.frame.buttonId == 0xE0)) {
          toggleRelay();
        }

        // skip any edges that might show up right after
        delayMs(500);

        m_somfy.clear();

        // will go back to LPM in main loop
    }

    void onReceiveFailed() final
    {
        // we're in interrupt context here

        // skip any edges that might show up right after
        delayMs(500);

        m_somfy.clear();

        // will go back to LPM in main loop
    }

private:
    void initRelay()
    {
        P1DIR |= BIT3;
        P1OUT &= ~BIT3;
    }

    void toggleRelay()
    {
        P1OUT |= BIT3;
        delayMs(100);
        P1OUT &= ~BIT3;
    }

    SomfyRtsReceiver& m_somfy;
};

SomfyRtsReceiver somfy;
SomfyRtsListenerImpl somfyListener(somfy);

int main()
{
    Wdt().disable();

    // set P3 for power saving
    P3DIR = 0xFF;
    P3OUT = 0;

    initFastClock();

    SomfyRtsReceiver somfy;
    somfy.init();
    somfy.enableReceive();

    while(1) {
        // LMP1 as we want SMCLK running
        LPM1;
    }
}
