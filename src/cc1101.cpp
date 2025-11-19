#include "cc1101.h"
#include "spi_driver.h"

#include <Arduino.h>

/****************************************************************/
constexpr uint8_t bytesInRxFifo = 0x7F;  // byte number in RXfifo
constexpr uint8_t maxModul = 6;

uint8_t modulation = 2;
uint8_t frend0;
uint8_t chan = 0;
int pa = 12;
uint8_t lastPa;
uint8_t gdo0;
uint8_t gdo2;
uint8_t gdo0M[maxModul];
uint8_t gdo2M[maxModul];
uint8_t gdoSet = 0;
bool ccMode = false;
float mhz = 433.92;
uint8_t m4RxBw = 0;
uint8_t m4DaRa;
uint8_t m2DcOff;
uint8_t m2ModFm;
uint8_t m2Manch;
uint8_t m2SyncM;
uint8_t m1Fec;
uint8_t m1Pre;
uint8_t m1Chsp;
uint8_t pc1Pqt;
uint8_t pc1CrcAf;
uint8_t pc1AppSt;
uint8_t pc1AdrChk;
uint8_t pc0Wdata;
uint8_t pc0PktForm;
uint8_t pc0CrcEn;
uint8_t pc0LenConf;
uint8_t trxState = 0;
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
/* minimal set of CC1101 methods used by src/somfy.cpp */

/* configure GDO pins */
void CC1101::gdoSetFunc()
{
    pinMode(gdo0, OUTPUT);
    pinMode(gdo2, INPUT);
}

/* set GDO pair */
void CC1101::setGdo(uint8_t gdo0Pin, uint8_t gdo2Pin)
{
    gdo0 = gdo0Pin;
    gdo2 = gdo2Pin;
    gdoSetFunc();
}

/* configure SPI pins for driver */
void CC1101::setSpiPin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss)
{
    spiDriver.setSpiPin(sck, miso, mosi, ss);
}

/* reset sequence */
void CC1101::reset()
{
    spiDriver.spiStart();
    digitalWrite(spiDriver.getSsPin(), LOW);
    delay(1);
    digitalWrite(spiDriver.getSsPin(), HIGH);
    delay(1);
    digitalWrite(spiDriver.getSsPin(), LOW);
    while (digitalRead(spiDriver.getMisoPin()));
    SPI.transfer(CC1101_SRES);
    while (digitalRead(spiDriver.getMisoPin()));
    digitalWrite(spiDriver.getSsPin(), HIGH);
    spiDriver.spiEnd();
}

/* initialize SPI and device (kept minimal) */
void CC1101::init()
{
    spiDriver.setSpiDefault();
    spiDriver.spiStart();
    digitalWrite(spiDriver.getSckPin(), HIGH);
    digitalWrite(spiDriver.getMosiPin(), LOW);
    reset();
    spiDriver.spiEnd();
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

/* set receive mode (used by somfy.cpp) */
void CC1101::setRx()
{
    spiDriver.spiStrobe(CC1101_SIDLE);
    spiDriver.spiStrobe(CC1101_SRX);
    trxState = 2;
}