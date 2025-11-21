#include "clock.h"

void initSlowClock()
{
    // select lowest DCOx and MODx settings
    DCOCTL = 0;

    // select calibrated frequency
    if (CALBC1_1MHZ != 0xFF)
    {
        DCOCTL = CALDCO_1MHZ;
        BCSCTL1 = CALBC1_1MHZ;
        BCSCTL2 = SELM_0 + DIVM_0 + DIVS_0;     // DCO, main = 16MHz, sub = 125kHz
        BCSCTL3 |= LFXT1S_2;                    // LFXT1 = VLO (12kHz)
    }
}

void initFastClock()
{
    /* 8MHz internal oscillator */

    // select lowest DCOx and MODx settings
    DCOCTL = 0;

    // select calibrated frequency
    if (CALBC1_8MHZ != 0xFF)
    {
        DCOCTL = CALDCO_8MHZ;
        BCSCTL1 = CALBC1_8MHZ;
        BCSCTL2 = SELM_0 + DIVM_0 + DIVS_0;     // DCO, main = 16MHz, sub = 125kHz
        BCSCTL3 |= LFXT1S_2;                    // LFXT1 = VLO (12kHz)
    }
}
