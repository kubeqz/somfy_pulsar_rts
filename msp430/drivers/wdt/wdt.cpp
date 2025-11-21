#include <drivers/wdt/wdt.h>
#include <msp430.h>

void Wdt::enable(WdtTimeout timeout)
{
    if (timeout == WdtTimeoutSlow) {
        WDTCTL = WDTPW+WDTCNTCL+WDTSSEL+WDTIS0;
    } else if (timeout == WdtTimeoutFast) {
        WDTCTL = WDTPW+WDTCNTCL+WDTSSEL+WDTIS1;
    }
}

void Wdt::disable()
{
    WDTCTL = WDTPW + WDTHOLD;   // stop watchdog timer
}
