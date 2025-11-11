#pragma once

#include <Arduino.h>

//***************************************CC1101 define**************************************************//
// CC1101 CONFIG REGISTER
constexpr uint8_t CC1101_IOCFG2       = 0x00; // GDO2 output pin configuration
constexpr uint8_t CC1101_IOCFG1       = 0x01; // GDO1 output pin configuration
constexpr uint8_t CC1101_IOCFG0       = 0x02; // GDO0 output pin configuration
constexpr uint8_t CC1101_FIFOTHR      = 0x03; // RX FIFO and TX FIFO thresholds
constexpr uint8_t CC1101_SYNC1        = 0x04; // Sync word, high uint8_t
constexpr uint8_t CC1101_SYNC0        = 0x05; // Sync word, low uint8_t
constexpr uint8_t CC1101_PKTLEN       = 0x06; // Packet length
constexpr uint8_t CC1101_PKTCTRL1     = 0x07; // Packet automation control
constexpr uint8_t CC1101_PKTCTRL0     = 0x08; // Packet automation control
constexpr uint8_t CC1101_ADDR         = 0x09; // Device address
constexpr uint8_t CC1101_CHANNR       = 0x0A; // Channel number
constexpr uint8_t CC1101_FSCTRL1      = 0x0B; // Frequency synthesizer control
constexpr uint8_t CC1101_FSCTRL0      = 0x0C; // Frequency synthesizer control
constexpr uint8_t CC1101_FREQ2        = 0x0D; // Frequency control word, high uint8_t
constexpr uint8_t CC1101_FREQ1        = 0x0E; // Frequency control word, middle uint8_t
constexpr uint8_t CC1101_FREQ0        = 0x0F; // Frequency control word, low uint8_t
constexpr uint8_t CC1101_MDMCFG4      = 0x10; // Modem configuration
constexpr uint8_t CC1101_MDMCFG3      = 0x11; // Modem configuration
constexpr uint8_t CC1101_MDMCFG2      = 0x12; // Modem configuration
constexpr uint8_t CC1101_MDMCFG1      = 0x13; // Modem configuration
constexpr uint8_t CC1101_MDMCFG0      = 0x14; // Modem configuration
constexpr uint8_t CC1101_DEVIATN      = 0x15; // Modem deviation setting
constexpr uint8_t CC1101_MCSM2        = 0x16; // Main Radio Control State Machine configuration
constexpr uint8_t CC1101_MCSM1        = 0x17; // Main Radio Control State Machine configuration
constexpr uint8_t CC1101_MCSM0        = 0x18; // Main Radio Control State Machine configuration
constexpr uint8_t CC1101_FOCCFG       = 0x19; // Frequency Offset Compensation configuration
constexpr uint8_t CC1101_BSCFG        = 0x1A; // Bit Synchronization configuration
constexpr uint8_t CC1101_AGCCTRL2     = 0x1B; // AGC control
constexpr uint8_t CC1101_AGCCTRL1     = 0x1C; // AGC control
constexpr uint8_t CC1101_AGCCTRL0     = 0x1D; // AGC control
constexpr uint8_t CC1101_WOREVT1      = 0x1E; // High uint8_t Event 0 timeout
constexpr uint8_t CC1101_WOREVT0      = 0x1F; // Low uint8_t Event 0 timeout
constexpr uint8_t CC1101_WORCTRL      = 0x20; // Wake On Radio control
constexpr uint8_t CC1101_FREND1       = 0x21; // Front end RX configuration
constexpr uint8_t CC1101_FREND0       = 0x22; // Front end TX configuration
constexpr uint8_t CC1101_FSCAL3       = 0x23; // Frequency synthesizer calibration
constexpr uint8_t CC1101_FSCAL2       = 0x24; // Frequency synthesizer calibration
constexpr uint8_t CC1101_FSCAL1       = 0x25; // Frequency synthesizer calibration
constexpr uint8_t CC1101_FSCAL0       = 0x26; // Frequency synthesizer calibration
constexpr uint8_t CC1101_RCCTRL1      = 0x27; // RC oscillator configuration
constexpr uint8_t CC1101_RCCTRL0      = 0x28; // RC oscillator configuration
constexpr uint8_t CC1101_FSTEST       = 0x29; // Frequency synthesizer calibration control
constexpr uint8_t CC1101_PTEST        = 0x2A; // Production test
constexpr uint8_t CC1101_AGCTEST      = 0x2B; // AGC test
constexpr uint8_t CC1101_TEST2        = 0x2C; // Various test settings
constexpr uint8_t CC1101_TEST1        = 0x2D; // Various test settings
constexpr uint8_t CC1101_TEST0        = 0x2E; // Various test settings

// CC1101 Strobe commands
constexpr uint8_t CC1101_SRES         = 0x30;
constexpr uint8_t CC1101_SFSTXON      = 0x31;
constexpr uint8_t CC1101_SXOFF        = 0x32;
constexpr uint8_t CC1101_SCAL         = 0x33;
constexpr uint8_t CC1101_SRX          = 0x34;
constexpr uint8_t CC1101_STX          = 0x35;
constexpr uint8_t CC1101_SIDLE        = 0x36;
constexpr uint8_t CC1101_SAFC         = 0x37;
constexpr uint8_t CC1101_SWOR         = 0x38;
constexpr uint8_t CC1101_SPWD         = 0x39;
constexpr uint8_t CC1101_SFRX         = 0x3A;
constexpr uint8_t CC1101_SFTX         = 0x3B;
constexpr uint8_t CC1101_SWORRST      = 0x3C;
constexpr uint8_t CC1101_SNOP         = 0x3D;

// CC1101 STATUS REGISTER
constexpr uint8_t CC1101_PARTNUM      = 0x30;
constexpr uint8_t CC1101_VERSION      = 0x31;
constexpr uint8_t CC1101_FREQEST      = 0x32;
constexpr uint8_t CC1101_LQI          = 0x33;
constexpr uint8_t CC1101_RSSI         = 0x34;
constexpr uint8_t CC1101_MARCSTATE    = 0x35;
constexpr uint8_t CC1101_WORTIME1     = 0x36;
constexpr uint8_t CC1101_WORTIME0     = 0x37;
constexpr uint8_t CC1101_PKTSTATUS    = 0x38;
constexpr uint8_t CC1101_VCO_VC_DAC   = 0x39;
constexpr uint8_t CC1101_TXBYTES      = 0x3A;
constexpr uint8_t CC1101_RXBYTES      = 0x3B;

// CC1101 PATABLE, TXFIFO, RXFIFO
constexpr uint8_t CC1101_PATABLE      = 0x3E;
constexpr uint8_t CC1101_TXFIFO       = 0x3F;
constexpr uint8_t CC1101_RXFIFO       = 0x3F;

//************************************* class **************************************************//
class CC1101
{
private:
    void spiStart();
    void spiEnd();
    void gdoSetFunc();
    void gdo0SetFunc();
    void reset();
    void setSpi();
    void regConfigSettings();
    void calibrate();
    void splitPktctrl0();
    void splitPktctrl1();
    void splitMdmcfg1();
    void splitMdmcfg2();
    void splitMdmcfg4();

public:
    void init();
    uint8_t spiReadStatus(uint8_t addr);
    void setSpiPin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss);
    void addSpiPin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss, uint8_t modul);
    void setGdo(uint8_t gdo0Pin, uint8_t gdo2Pin);
    void setGdo0(uint8_t gdo0Pin);
    void addGdo(uint8_t gdo0Pin, uint8_t gdo2Pin, uint8_t modul);
    void addGdo0(uint8_t gdo0Pin, uint8_t modul);
    void setModul(uint8_t modul);
    void setCcMode(bool s);
    void setModulation(uint8_t m);
    void setPa(int p);
    void setMhz(float mhz);
    void setChannel(uint8_t chnl);
    void setChsp(float f);
    void setRxBw(float f);
    void setDRate(float d);
    void setDeviation(float d);
    void setTx();
    void setRx();
    void setTx(float mhz);
    void setRx(float mhz);
    int getRssi();
    uint8_t getLqi();
    void setSres();
    void setSidle();
    void goSleep();
    void sendData(uint8_t *txBuffer, uint8_t size);
    void sendData(char *txchar);
    void sendData(uint8_t *txBuffer, uint8_t size, int t);
    void sendData(char *txchar, int t);
    uint8_t checkReceiveFlag();
    uint8_t receiveData(uint8_t *rxBuffer);
    bool checkCrc();
    void spiStrobe(uint8_t strobe);
    void spiWriteReg(uint8_t addr, uint8_t value);
    void spiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num);
    uint8_t spiReadReg(uint8_t addr);
    void spiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num);
    void setClb(uint8_t b, uint8_t s, uint8_t e);
    bool getCc1101();
    uint8_t getMode();
    void setSyncWord(uint8_t sh, uint8_t sl);
    void setAddr(uint8_t v);
    void setWhiteData(bool v);
    void setPktFormat(uint8_t v);
    void setCrc(bool v);
    void setLengthConfig(uint8_t v);
    void setPacketLength(uint8_t v);
    void setDcFilterOff(bool v);
    void setManchester(bool v);
    void setSyncMode(uint8_t v);
    void setFec(bool v);
    void setPre(uint8_t v);
    void setPqt(uint8_t v);
    void setCrcAf(bool v);
    void setAppendStatus(bool v);
    void setAdrChk(uint8_t v);
    bool checkRxFifo(int t);
};