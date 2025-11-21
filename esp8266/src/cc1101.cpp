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
void CC1101::gdoSetFunc()
{
    pinMode(gdo0, OUTPUT);
    pinMode(gdo2, INPUT);
}

void CC1101::gdo0SetFunc()
{
    pinMode(gdo0, INPUT);
}

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

void CC1101::init()
{
    spiDriver.setSpiDefault();
    spiDriver.spiStart();
    digitalWrite(spiDriver.getSckPin(), HIGH);
    digitalWrite(spiDriver.getMosiPin(), LOW);
    reset();
    regConfigSettings();
    spiDriver.spiEnd();
}

void CC1101::setSpiPin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss)
{
    spiDriver.setSpiPin(sck, miso, mosi, ss);
}

void CC1101::addSpiPin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss, uint8_t modul)
{
    // Store pins for multi-module support if needed
    // This would require extending SpiDriver for module management
}

void CC1101::setGdo(uint8_t gdo0Pin, uint8_t gdo2Pin)
{
    gdo0 = gdo0Pin;
    gdo2 = gdo2Pin;
    gdoSetFunc();
}

void CC1101::setGdo0(uint8_t gdo0Pin)
{
    gdo0 = gdo0Pin;
    gdo0SetFunc();
}

void CC1101::addGdo(uint8_t gdo0Pin, uint8_t gdo2Pin, uint8_t modul)
{
    gdo0M[modul] = gdo0Pin;
    gdo2M[modul] = gdo2Pin;
    gdoSet = 2;
    gdoSetFunc();
}

void CC1101::addGdo0(uint8_t gdo0Pin, uint8_t modul)
{
    gdo0M[modul] = gdo0Pin;
    gdoSet = 1;
    gdo0SetFunc();
}

void CC1101::setModul(uint8_t modul)
{
    if (gdoSet == 1)
    {
        gdo0 = gdo0M[modul];
    }
    else if (gdoSet == 2)
    {
        gdo0 = gdo0M[modul];
        gdo2 = gdo2M[modul];
    }
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

void CC1101::setClb(uint8_t b, uint8_t s, uint8_t e)
{
    if (b == 1)
    {
        clb1[0] = s;
        clb1[1] = e;
    }
    else if (b == 2)
    {
        clb2[0] = s;
        clb2[1] = e;
    }
    else if (b == 3)
    {
        clb3[0] = s;
        clb3[1] = e;
    }
    else if (b == 4)
    {
        clb4[0] = s;
        clb4[1] = e;
    }
}

bool CC1101::getCc1101()
{
    if (spiDriver.spiReadStatus(0x31) > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t CC1101::getMode()
{
    return trxState;
}

void CC1101::setSyncWord(uint8_t sh, uint8_t sl)
{
    spiDriver.spiWriteReg(CC1101_SYNC1, sh);
    spiDriver.spiWriteReg(CC1101_SYNC0, sl);
}

void CC1101::setAddr(uint8_t v)
{
    spiDriver.spiWriteReg(CC1101_ADDR, v);
}

void CC1101::setPqt(uint8_t v)
{
    splitPktctrl1();
    pc1Pqt = 0;
    if (v > 7)
    {
        v = 7;
    }
    pc1Pqt = v * 32;
    spiDriver.spiWriteReg(CC1101_PKTCTRL1, pc1Pqt + pc1CrcAf + pc1AppSt + pc1AdrChk);
}

void CC1101::setCrcAf(bool v)
{
    splitPktctrl1();
    pc1CrcAf = 0;
    if (v)
    {
        pc1CrcAf = 8;
    }
    spiDriver.spiWriteReg(CC1101_PKTCTRL1, pc1Pqt + pc1CrcAf + pc1AppSt + pc1AdrChk);
}

void CC1101::setAppendStatus(bool v)
{
    splitPktctrl1();
    pc1AppSt = 0;
    if (v)
    {
        pc1AppSt = 4;
    }
    spiDriver.spiWriteReg(CC1101_PKTCTRL1, pc1Pqt + pc1CrcAf + pc1AppSt + pc1AdrChk);
}

void CC1101::setAdrChk(uint8_t v)
{
    splitPktctrl1();
    pc1AdrChk = 0;
    if (v > 3)
    {
        v = 3;
    }
    pc1AdrChk = v;
    spiDriver.spiWriteReg(CC1101_PKTCTRL1, pc1Pqt + pc1CrcAf + pc1AppSt + pc1AdrChk);
}

void CC1101::setWhiteData(bool v)
{
    splitPktctrl0();
    pc0Wdata = 0;
    if (v)
    {
        pc0Wdata = 64;
    }
    spiDriver.spiWriteReg(CC1101_PKTCTRL0, pc0Wdata + pc0PktForm + pc0CrcEn + pc0LenConf);
}

void CC1101::setPktFormat(uint8_t v)
{
    splitPktctrl0();
    pc0PktForm = 0;
    if (v > 3)
    {
        v = 3;
    }
    pc0PktForm = v * 16;
    spiDriver.spiWriteReg(CC1101_PKTCTRL0, pc0Wdata + pc0PktForm + pc0CrcEn + pc0LenConf);
}

void CC1101::setCrc(bool v)
{
    splitPktctrl0();
    pc0CrcEn = 0;
    if (v)
    {
        pc0CrcEn = 4;
    }
    spiDriver.spiWriteReg(CC1101_PKTCTRL0, pc0Wdata + pc0PktForm + pc0CrcEn + pc0LenConf);
}

void CC1101::setLengthConfig(uint8_t v)
{
    splitPktctrl0();
    pc0LenConf = 0;
    if (v > 3)
    {
        v = 3;
    }
    pc0LenConf = v;
    spiDriver.spiWriteReg(CC1101_PKTCTRL0, pc0Wdata + pc0PktForm + pc0CrcEn + pc0LenConf);
}

void CC1101::setPacketLength(uint8_t v)
{
    spiDriver.spiWriteReg(CC1101_PKTLEN, v);
}

void CC1101::setDcFilterOff(bool v)
{
    splitMdmcfg2();
    m2DcOff = 0;
    if (v)
    {
        m2DcOff = 128;
    }
    spiDriver.spiWriteReg(CC1101_MDMCFG2, m2DcOff + m2ModFm + m2Manch + m2SyncM);
}

void CC1101::setManchester(bool v)
{
    splitMdmcfg2();
    m2Manch = 0;
    if (v)
    {
        m2Manch = 8;
    }
    spiDriver.spiWriteReg(CC1101_MDMCFG2, m2DcOff + m2ModFm + m2Manch + m2SyncM);
}

void CC1101::setSyncMode(uint8_t v)
{
    splitMdmcfg2();
    m2SyncM = 0;
    if (v > 7)
    {
        v = 7;
    }
    m2SyncM = v;
    spiDriver.spiWriteReg(CC1101_MDMCFG2, m2DcOff + m2ModFm + m2Manch + m2SyncM);
}

void CC1101::setFec(bool v)
{
    splitMdmcfg1();
    m1Fec = 0;
    if (v)
    {
        m1Fec = 128;
    }
    spiDriver.spiWriteReg(CC1101_MDMCFG1, m1Fec + m1Pre + m1Chsp);
}

void CC1101::setPre(uint8_t v)
{
    splitMdmcfg1();
    m1Pre = 0;
    if (v > 7)
    {
        v = 7;
    }
    m1Pre = v * 16;
    spiDriver.spiWriteReg(CC1101_MDMCFG1, m1Fec + m1Pre + m1Chsp);
}

void CC1101::setChannel(uint8_t ch)
{
    chan = ch;
    spiDriver.spiWriteReg(CC1101_CHANNR, chan);
}

void CC1101::setChsp(float f)
{
    splitMdmcfg1();
    uint8_t mdmcfg0 = 0;
    m1Chsp = 0;
    if (f > 405.456543f)
    {
        f = 405.456543f;
    }
    if (f < 25.390625f)
    {
        f = 25.390625f;
    }
    for (int i = 0; i < 5; i++)
    {
        if (f <= 50.682068f)
        {
            f -= 25.390625f;
            f /= 0.0991825f;
            mdmcfg0 = static_cast<uint8_t>(f);
            float s1 = (f - mdmcfg0) * 10.0f;
            if (s1 >= 5.0f)
            {
                mdmcfg0++;
            }
            i = 5;
        }
        else
        {
            m1Chsp++;
            f /= 2.0f;
        }
    }
    spiDriver.spiWriteReg(19, m1Chsp + m1Fec + m1Pre);
    spiDriver.spiWriteReg(20, mdmcfg0);
}

void CC1101::setRxBw(float f)
{
    splitMdmcfg4();
    int s1 = 3;
    int s2 = 3;
    for (int i = 0; i < 3; i++)
    {
        if (f > 101.5625f)
        {
            f /= 2.0f;
            s1--;
        }
        else
        {
            i = 3;
        }
    }
    for (int i = 0; i < 3; i++)
    {
        if (f > 58.1f)
        {
            f /= 1.25f;
            s2--;
        }
        else
        {
            i = 3;
        }
    }
    s1 *= 64;
    s2 *= 16;
    m4RxBw = static_cast<uint8_t>(s1 + s2);
    spiDriver.spiWriteReg(16, m4RxBw + m4DaRa);
}

void CC1101::setDRate(float d)
{
    splitMdmcfg4();
    float c = d;
    uint8_t mdmcfg3 = 0;
    if (c > 1621.83f)
    {
        c = 1621.83f;
    }
    if (c < 0.0247955f)
    {
        c = 0.0247955f;
    }
    m4DaRa = 0;
    for (int i = 0; i < 20; i++)
    {
        if (c <= 0.0494942f)
        {
            c = c - 0.0247955f;
            c = c / 0.00009685f;
            mdmcfg3 = static_cast<uint8_t>(c);
            float s1 = (c - mdmcfg3) * 10.0f;
            if (s1 >= 5.0f)
            {
                mdmcfg3++;
            }
            i = 20;
        }
        else
        {
            m4DaRa++;
            c = c / 2.0f;
        }
    }
    spiDriver.spiWriteReg(16, m4RxBw + m4DaRa);
    spiDriver.spiWriteReg(17, mdmcfg3);
}

void CC1101::setDeviation(float d)
{
    float f = 1.586914f;
    float v = 0.19836425f;
    int c = 0;
    if (d > 380.859375f)
    {
        d = 380.859375f;
    }
    if (d < 1.586914f)
    {
        d = 1.586914f;
    }
    for (int i = 0; i < 255; i++)
    {
        f += v;
        if (c == 7)
        {
            v *= 2.0f;
            c = -1;
            i += 8;
        }
        if (f >= d)
        {
            c = i;
            i = 255;
        }
        c++;
    }
    spiDriver.spiWriteReg(21, static_cast<uint8_t>(c));
}

void CC1101::splitPktctrl1()
{
    int calc = spiDriver.spiReadStatus(7);
    pc1Pqt = 0;
    pc1CrcAf = 0;
    pc1AppSt = 0;
    pc1AdrChk = 0;
    for (bool i = false; i == false;)
    {
        if (calc >= 32)
        {
            calc -= 32;
            pc1Pqt += 32;
        }
        else if (calc >= 8)
        {
            calc -= 8;
            pc1CrcAf += 8;
        }
        else if (calc >= 4)
        {
            calc -= 4;
            pc1AppSt += 4;
        }
        else
        {
            pc1AdrChk = calc;
            i = true;
        }
    }
}

void CC1101::splitPktctrl0()
{
    int calc = spiDriver.spiReadStatus(8);
    pc0Wdata = 0;
    pc0PktForm = 0;
    pc0CrcEn = 0;
    pc0LenConf = 0;
    for (bool i = false; i == false;)
    {
        if (calc >= 64)
        {
            calc -= 64;
            pc0Wdata += 64;
        }
        else if (calc >= 16)
        {
            calc -= 16;
            pc0PktForm += 16;
        }
        else if (calc >= 4)
        {
            calc -= 4;
            pc0CrcEn += 4;
        }
        else
        {
            pc0LenConf = calc;
            i = true;
        }
    }
}

void CC1101::splitMdmcfg1()
{
    int calc = spiDriver.spiReadStatus(19);
    m1Fec = 0;
    m1Pre = 0;
    m1Chsp = 0;
    for (bool i = false; i == false;)
    {
        if (calc >= 128)
        {
            calc -= 128;
            m1Fec += 128;
        }
        else if (calc >= 16)
        {
            calc -= 16;
            m1Pre += 16;
        }
        else
        {
            m1Chsp = calc;
            i = true;
        }
    }
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

void CC1101::splitMdmcfg4()
{
    int calc = spiDriver.spiReadStatus(16);
    m4RxBw = 0;
    m4DaRa = 0;
    for (bool i = false; i == false;)
    {
        if (calc >= 64)
        {
            calc -= 64;
            m4RxBw += 64;
        }
        else if (calc >= 16)
        {
            calc -= 16;
            m4RxBw += 16;
        }
        else
        {
            m4DaRa = calc;
            i = true;
        }
    }
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

void CC1101::setTx()
{
    spiDriver.spiStrobe(CC1101_SIDLE);
    spiDriver.spiStrobe(CC1101_STX);
    trxState = 1;
}

void CC1101::setRx()
{
    spiDriver.spiStrobe(CC1101_SIDLE);
    spiDriver.spiStrobe(CC1101_SRX);
    trxState = 2;
}

void CC1101::setTx(float mhzVal)
{
    spiDriver.spiStrobe(CC1101_SIDLE);
    setMhz(mhzVal);
    spiDriver.spiStrobe(CC1101_STX);
    trxState = 1;
}

void CC1101::setRx(float mhzVal)
{
    spiDriver.spiStrobe(CC1101_SIDLE);
    setMhz(mhzVal);
    spiDriver.spiStrobe(CC1101_SRX);
    trxState = 2;
}

int CC1101::getRssi()
{
    int rssi = spiDriver.spiReadStatus(CC1101_RSSI);
    if (rssi >= 128)
    {
        rssi = (rssi - 256) / 2 - 74;
    }
    else
    {
        rssi = (rssi / 2) - 74;
    }
    return rssi;
}

uint8_t CC1101::getLqi()
{
    uint8_t lqi = static_cast<uint8_t>(spiDriver.spiReadStatus(CC1101_LQI));
    return lqi;
}

void CC1101::setSres()
{
    spiDriver.spiStrobe(CC1101_SRES);
    trxState = 0;
}

void CC1101::setSidle()
{
    spiDriver.spiStrobe(CC1101_SIDLE);
    trxState = 0;
}

void CC1101::goSleep()
{
    trxState = 0;
    spiDriver.spiStrobe(0x36);
    spiDriver.spiStrobe(0x39);
}

void CC1101::sendData(char *txchar)
{
    int len = strlen(txchar);
    uint8_t chartobyte[len];
    for (int i = 0; i < len; i++)
    {
        chartobyte[i] = static_cast<uint8_t>(txchar[i]);
    }
    sendData(chartobyte, static_cast<uint8_t>(len));
}

void CC1101::sendData(uint8_t *txBuffer, uint8_t size)
{
    spiDriver.spiWriteReg(CC1101_TXFIFO, size);
    spiDriver.spiWriteBurstReg(CC1101_TXFIFO, txBuffer, size);
    spiDriver.spiStrobe(CC1101_SIDLE);
    spiDriver.spiStrobe(CC1101_STX);
    while (!digitalRead(gdo0));
    while (digitalRead(gdo0));
    spiDriver.spiStrobe(CC1101_SFTX);
    trxState = 1;
}

void CC1101::sendData(char *txchar, int t)
{
    int len = strlen(txchar);
    uint8_t chartobyte[len];
    for (int i = 0; i < len; i++)
    {
        chartobyte[i] = static_cast<uint8_t>(txchar[i]);
    }
    sendData(chartobyte, static_cast<uint8_t>(len), t);
}

void CC1101::sendData(uint8_t *txBuffer, uint8_t size, int t)
{
    spiDriver.spiWriteReg(CC1101_TXFIFO, size);
    spiDriver.spiWriteBurstReg(CC1101_TXFIFO, txBuffer, size);
    spiDriver.spiStrobe(CC1101_SIDLE);
    spiDriver.spiStrobe(CC1101_STX);
    delay(t);
    spiDriver.spiStrobe(CC1101_SFTX);
    trxState = 1;
}

bool CC1101::checkCrc()
{
    uint8_t lqi = static_cast<uint8_t>(spiDriver.spiReadStatus(CC1101_LQI));
    bool crcOk = bitRead(lqi, 7);
    if (crcOk)
    {
        return true;
    }
    else
    {
        spiDriver.spiStrobe(CC1101_SFRX);
        spiDriver.spiStrobe(CC1101_SRX);
        return false;
    }
}

bool CC1101::checkRxFifo(int t)
{
    if (trxState != 2)
    {
        setRx();
    }
    if (spiDriver.spiReadStatus(CC1101_RXBYTES) & bytesInRxFifo)
    {
        delay(t);
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t CC1101::checkReceiveFlag()
{
    if (trxState != 2)
    {
        setRx();
    }
    if (digitalRead(gdo0))
    {
        while (digitalRead(gdo0));
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t CC1101::receiveData(uint8_t *rxBuffer)
{
    uint8_t size;
    uint8_t status[2];

    if (spiDriver.spiReadStatus(CC1101_RXBYTES) & bytesInRxFifo)
    {
        size = spiDriver.spiReadReg(CC1101_RXFIFO);
        spiDriver.spiReadBurstReg(CC1101_RXFIFO, rxBuffer, size);
        spiDriver.spiReadBurstReg(CC1101_RXFIFO, status, 2);
        spiDriver.spiStrobe(CC1101_SFRX);
        spiDriver.spiStrobe(CC1101_SRX);
        return size;
    }
    else
    {
        spiDriver.spiStrobe(CC1101_SFRX);
        spiDriver.spiStrobe(CC1101_SRX);
        return 0;
    }
}