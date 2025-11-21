#include "drivers/cc1101/cc1101.h"
#include "drivers/spi/spi_driver.h"

uint8_t modulation = 2;
uint8_t frend0;
uint8_t chan = 0;
int pa = 12;
uint8_t lastPa;
float mhz = 433.92;
bool ccMode = false;
uint8_t m4RxBw = 0;
uint8_t m4DaRa;
uint8_t m2DcOff;
uint8_t m2ModFm;
uint8_t m2Manch;
uint8_t m2SyncM;
uint8_t clb1[2] = {24, 28};
uint8_t clb2[2] = {31, 38};
uint8_t clb3[2] = {65, 76};
uint8_t clb4[2] = {77, 79};

/****************************************************************/
uint8_t paTable[8]{0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t paTable315[8]{0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2};
uint8_t paTable433[8]{0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0};
uint8_t paTable868[10]{0x03, 0x17, 0x1D, 0x26, 0x37, 0x50, 0x86, 0xCD, 0xC5, 0xC0};
uint8_t paTable915[10]{0x03, 0x0E, 0x1E, 0x27, 0x38, 0x8E, 0x84, 0xCC, 0xC3, 0xC0};

/****************************************************************/
static SpiDriver spiDriver;

/****************************************************************/

static void delay(uint32_t delay)
{
    while (delay--) {
        asm volatile(" NOP");
    }
}

/* reset sequence */
void CC1101::reset()
{
    spiDriver.spiStart();
    spiDriver.setSs(false);
    delay(0xF);
    spiDriver.setSs(true);
    delay(0xF);
    spiDriver.setSs(false);
    delay(0xFF);
    spiDriver.transferByte(CC1101_SRES);
    delay(0xFF);
    spiDriver.setSs(true);
    regConfigSettings();
}

/* initialize SPI and device (kept minimal) */
void CC1101::init()
{
    reset();
}

void CC1101::regConfigSettings()
{
    spiDriver.spiWriteReg(CC1101_FSCTRL1, 0x06);

    setCcMode(ccMode);
    setMhz(mhz);

    spiDriver.spiWriteReg(CC1101_MDMCFG1, 0x02);
    spiDriver.spiWriteReg(CC1101_MDMCFG0, 0xF8);
    spiDriver.spiWriteReg(CC1101_CHANNR, chan);
    spiDriver.spiWriteReg(CC1101_DEVIATN, 0x47);
    spiDriver.spiWriteReg(CC1101_FREND1, 0x56);
    spiDriver.spiWriteReg(CC1101_MCSM0, 0x18);
    spiDriver.spiWriteReg(CC1101_FOCCFG, 0x16);
    spiDriver.spiWriteReg(CC1101_BSCFG, 0x1C);
    spiDriver.spiWriteReg(CC1101_AGCCTRL2, 0xC7);
    spiDriver.spiWriteReg(CC1101_AGCCTRL1, 0x00);
    spiDriver.spiWriteReg(CC1101_AGCCTRL0, 0xB2);
    spiDriver.spiWriteReg(CC1101_FSCAL3, 0xE9);
    spiDriver.spiWriteReg(CC1101_FSCAL2, 0x2A);
    spiDriver.spiWriteReg(CC1101_FSCAL1, 0x00);
    spiDriver.spiWriteReg(CC1101_FSCAL0, 0x1F);
    spiDriver.spiWriteReg(CC1101_FSTEST, 0x59);
    spiDriver.spiWriteReg(CC1101_TEST2, 0x81);
    spiDriver.spiWriteReg(CC1101_TEST1, 0x35);
    spiDriver.spiWriteReg(CC1101_TEST0, 0x09);
    spiDriver.spiWriteReg(CC1101_PKTCTRL1, 0x04);
    spiDriver.spiWriteReg(CC1101_ADDR, 0x00);
    spiDriver.spiWriteReg(CC1101_PKTLEN, 0x00);
}

void CC1101::setCcMode(bool s)
{
    ccMode = s;
    if (ccMode)
    {
        spiDriver.spiWriteReg(CC1101_IOCFG2, 0x0B);
        spiDriver.spiWriteReg(CC1101_IOCFG0, 0x06);
        spiDriver.spiWriteReg(CC1101_PKTCTRL0, 0x05);
        spiDriver.spiWriteReg(CC1101_MDMCFG3, 0xF8);
        spiDriver.spiWriteReg(CC1101_MDMCFG4, 11 + m4RxBw);
    }
    else
    {
        spiDriver.spiWriteReg(CC1101_IOCFG2, 0x0D);
        spiDriver.spiWriteReg(CC1101_IOCFG0, 0x0D);
        spiDriver.spiWriteReg(CC1101_PKTCTRL0, 0x32);
        spiDriver.spiWriteReg(CC1101_MDMCFG3, 0x93);
        spiDriver.spiWriteReg(CC1101_MDMCFG4, 7 + m4RxBw);
    }
    setModulation(modulation);
}


void CC1101::setModulation(uint8_t m)
{
    if (m > 4)
    {
        m = 4;
    }
    modulation = m;
    splitMdmcfg2();
    switch (m)
    {
    case 0:
        m2ModFm = 0x00;
        frend0 = 0x10;
        break; // 2-FSK
    case 1:
        m2ModFm = 0x10;
        frend0 = 0x10;
        break; // GFSK
    case 2:
        m2ModFm = 0x30;
        frend0 = 0x11;
        break; // ASK
    case 3:
        m2ModFm = 0x40;
        frend0 = 0x10;
        break; // 4-FSK
    case 4:
        m2ModFm = 0x70;
        frend0 = 0x10;
        break; // MSK
    }
    spiDriver.spiWriteReg(CC1101_MDMCFG2, m2DcOff + m2ModFm + m2Manch + m2SyncM);
    spiDriver.spiWriteReg(CC1101_FREND0, frend0);
    setPa(pa);
}

/* set operating frequency (keeps calibrate and setPa helpers) */
void CC1101::setMhz(float mhzVal)
{
    uint8_t freq2 = 0;
    uint8_t freq1 = 0;
    uint8_t freq0 = 0;

    mhz = mhzVal;

    for (bool i = false; i == false;)
    {
        if (mhz >= 26)
        {
            mhz -= 26;
            freq2 += 1;
        }
        else if (mhz >= 0.1015625f)
        {
            mhz -= 0.1015625f;
            freq1 += 1;
        }
        else if (mhz >= 0.00039675f)
        {
            mhz -= 0.00039675f;
            freq0 += 1;
        }
        else
        {
            i = true;
        }
    }
    if (freq0 > 255)
    {
        freq1 += 1;
        freq0 -= 256;
    }

    spiDriver.spiWriteReg(CC1101_FREQ2, freq2);
    spiDriver.spiWriteReg(CC1101_FREQ1, freq1);
    spiDriver.spiWriteReg(CC1101_FREQ0, freq0);

    calibrate();
}

/* calibrate helpers (used by setMhz) */
void CC1101::calibrate()
{
    if (mhz >= 300 && mhz <= 348)
    {
        spiDriver.spiWriteReg(CC1101_FSCTRL0, map(mhz, 300, 348, clb1[0], clb1[1]));
        if (mhz < 322.88f)
        {
            spiDriver.spiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            spiDriver.spiWriteReg(CC1101_TEST0, 0x09);
            int s = spiDriver.spiReadStatus(CC1101_FSCAL2);
            if (s < 32)
            {
                spiDriver.spiWriteReg(CC1101_FSCAL2, s + 32);
            }
            if (lastPa != 1)
            {
                setPa(pa);
            }
        }
    }
    else if (mhz >= 378 && mhz <= 464)
    {
        spiDriver.spiWriteReg(CC1101_FSCTRL0, map(mhz, 378, 464, clb2[0], clb2[1]));
        if (mhz < 430.5f)
        {
            spiDriver.spiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            spiDriver.spiWriteReg(CC1101_TEST0, 0x09);
            int s = spiDriver.spiReadStatus(CC1101_FSCAL2);
            if (s < 32)
            {
                spiDriver.spiWriteReg(CC1101_FSCAL2, s + 32);
            }
            if (lastPa != 2)
            {
                setPa(pa);
            }
        }
    }
    else if (mhz >= 779 && mhz <= 899.99)
    {
        spiDriver.spiWriteReg(CC1101_FSCTRL0, map(mhz, 779, 899, clb3[0], clb3[1]));
        if (mhz < 861)
        {
            spiDriver.spiWriteReg(CC1101_TEST0, 0x0B);
        }
        else
        {
            spiDriver.spiWriteReg(CC1101_TEST0, 0x09);
            int s = spiDriver.spiReadStatus(CC1101_FSCAL2);
            if (s < 32)
            {
                spiDriver.spiWriteReg(CC1101_FSCAL2, s + 32);
            }
            if (lastPa != 3)
            {
                setPa(pa);
            }
        }
    }
    else if (mhz >= 900 && mhz <= 928)
    {
        spiDriver.spiWriteReg(CC1101_FSCTRL0, map(mhz, 900, 928, clb4[0], clb4[1]));
        spiDriver.spiWriteReg(CC1101_TEST0, 0x09);
        int s = spiDriver.spiReadStatus(CC1101_FSCAL2);
        if (s < 32)
        {
            spiDriver.spiWriteReg(CC1101_FSCAL2, s + 32);
        }
        if (lastPa != 4)
        {
            setPa(pa);
        }
    }
}

/* set PA table (used by calibrate) */
void CC1101::setPa(int p)
{
    int a = 0;
    pa = p;

    if (mhz >= 300 && mhz <= 348)
    {
        if (pa <= -30)
            a = paTable315[0];
        else if (pa > -30 && pa <= -20)
            a = paTable315[1];
        else if (pa > -20 && pa <= -15)
            a = paTable315[2];
        else if (pa > -15 && pa <= -10)
            a = paTable315[3];
        else if (pa > -10 && pa <= 0)
            a = paTable315[4];
        else if (pa > 0 && pa <= 5)
            a = paTable315[5];
        else if (pa > 5 && pa <= 7)
            a = paTable315[6];
        else if (pa > 7)
            a = paTable315[7];
        lastPa = 1;
    }
    else if (mhz >= 378 && mhz <= 464)
    {
        if (pa <= -30)
            a = paTable433[0];
        else if (pa > -30 && pa <= -20)
            a = paTable433[1];
        else if (pa > -20 && pa <= -15)
            a = paTable433[2];
        else if (pa > -15 && pa <= -10)
            a = paTable433[3];
        else if (pa > -10 && pa <= 0)
            a = paTable433[4];
        else if (pa > 0 && pa <= 5)
            a = paTable433[5];
        else if (pa > 5 && pa <= 7)
            a = paTable433[6];
        else if (pa > 7)
            a = paTable433[7];
        lastPa = 2;
    }
    else if (mhz >= 779 && mhz <= 899.99)
    {
        if (pa <= -30)
            a = paTable868[0];
        else if (pa > -30 && pa <= -20)
            a = paTable868[1];
        else if (pa > -20 && pa <= -15)
            a = paTable868[2];
        else if (pa > -15 && pa <= -10)
            a = paTable868[3];
        else if (pa > -10 && pa <= -6)
            a = paTable868[4];
        else if (pa > -6 && pa <= 0)
            a = paTable868[5];
        else if (pa > 0 && pa <= 5)
            a = paTable868[6];
        else if (pa > 5 && pa <= 7)
            a = paTable868[7];
        else if (pa > 7 && pa <= 10)
            a = paTable868[8];
        else if (pa > 10)
            a = paTable868[9];
        lastPa = 3;
    }
    else if (mhz >= 900 && mhz <= 928)
    {
        if (pa <= -30)
            a = paTable915[0];
        else if (pa > -30 && pa <= -20)
            a = paTable915[1];
        else if (pa > -20 && pa <= -15)
            a = paTable915[2];
        else if (pa > -15 && pa <= -10)
            a = paTable915[3];
        else if (pa > -10 && pa <= -6)
            a = paTable915[4];
        else if (pa > -6 && pa <= 0)
            a = paTable915[5];
        else if (pa > 0 && pa <= 5)
            a = paTable915[6];
        else if (pa > 5 && pa <= 7)
            a = paTable915[7];
        else if (pa > 7 && pa <= 10)
            a = paTable915[8];
        else if (pa > 10)
            a = paTable915[9];
        lastPa = 4;
    }

    if (modulation == 2)
    {
        paTable[0] = 0;
        paTable[1] = static_cast<uint8_t>(a);
    }
    else
    {
        paTable[0] = static_cast<uint8_t>(a);
        paTable[1] = 0;
    }
    spiDriver.spiWriteBurstReg(CC1101_PATABLE, paTable, 8);
}


void CC1101::splitMdmcfg2()
{
    int calc = spiDriver.spiReadStatus(18);
    m2DcOff = 0;
    m2ModFm = 0;
    m2Manch = 0;
    m2SyncM = 0;
    for (bool i = false; i == false;)
    {
        if (calc >= 128)
        {
            calc -= 128;
            m2DcOff += 128;
        }
        else if (calc >= 16)
        {
            calc -= 16;
            m2ModFm += 16;
        }
        else if (calc >= 8)
        {
            calc -= 8;
            m2Manch += 8;
        }
        else
        {
            m2SyncM = calc;
            i = true;
        }
    }
}

void CC1101::setRx()
{
    spiDriver.spiStrobe(CC1101_SIDLE);
    spiDriver.spiStrobe(CC1101_SRX);
}

long CC1101::map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
