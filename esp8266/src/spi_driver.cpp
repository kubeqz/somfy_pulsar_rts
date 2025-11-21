#include "spi_driver.h"

constexpr uint8_t writeBurst = 0x40;   // write burst
constexpr uint8_t readSingle = 0x80;   // read single
constexpr uint8_t readBurst = 0xC0;    // read burst

SpiDriver::SpiDriver() : sckPin(13), misoPin(12), mosiPin(11), ssPin(10)
{
    setSpiDefault();
}

void SpiDriver::setSpiDefault()
{
#if defined __AVR_ATmega168__ || defined __AVR_ATmega328P__
    sckPin = 13;
    misoPin = 12;
    mosiPin = 11;
    ssPin = 10;
#elif defined __AVR_ATmega1280__ || defined __AVR_ATmega2560__
    sckPin = 52;
    misoPin = 50;
    mosiPin = 51;
    ssPin = 53;
#elif ESP8266
    sckPin = 14;
    misoPin = 12;
    mosiPin = 13;
    ssPin = 15;
#elif ESP32
    sckPin = 18;
    misoPin = 19;
    mosiPin = 23;
    ssPin = 5;
#endif
}

void SpiDriver::setSpiPin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss)
{
    sckPin = sck;
    misoPin = miso;
    mosiPin = mosi;
    ssPin = ss;
}

void SpiDriver::spiStart()
{
    pinMode(sckPin, OUTPUT);
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);
    pinMode(ssPin, OUTPUT);

#ifdef ESP32
    SPI.begin(sckPin, misoPin, mosiPin, ssPin);
#else
    SPI.begin();
#endif
}

void SpiDriver::spiEnd()
{
    SPI.endTransaction();
    SPI.end();
}

void SpiDriver::spiWriteReg(uint8_t addr, uint8_t value)
{
    spiStart();
    digitalWrite(ssPin, LOW);
    while (digitalRead(misoPin));
    SPI.transfer(addr);
    SPI.transfer(value);
    digitalWrite(ssPin, HIGH);
    spiEnd();
}

void SpiDriver::spiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num)
{
    uint8_t temp = addr | writeBurst;
    spiStart();
    digitalWrite(ssPin, LOW);
    while (digitalRead(misoPin));
    SPI.transfer(temp);
    for (uint8_t i = 0; i < num; i++)
    {
        SPI.transfer(buffer[i]);
    }
    digitalWrite(ssPin, HIGH);
    spiEnd();
}

void SpiDriver::spiStrobe(uint8_t strobe)
{
    spiStart();
    digitalWrite(ssPin, LOW);
    while (digitalRead(misoPin));
    SPI.transfer(strobe);
    digitalWrite(ssPin, HIGH);
    spiEnd();
}

uint8_t SpiDriver::spiReadReg(uint8_t addr)
{
    uint8_t temp = addr | readSingle;
    uint8_t value;
    spiStart();
    digitalWrite(ssPin, LOW);
    while (digitalRead(misoPin));
    SPI.transfer(temp);
    value = SPI.transfer(0);
    digitalWrite(ssPin, HIGH);
    spiEnd();
    return value;
}

void SpiDriver::spiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num)
{
    uint8_t temp = addr | readBurst;
    spiStart();
    digitalWrite(ssPin, LOW);
    while (digitalRead(misoPin));
    SPI.transfer(temp);
    for (uint8_t i = 0; i < num; i++)
    {
        buffer[i] = SPI.transfer(0);
    }
    digitalWrite(ssPin, HIGH);
    spiEnd();
}

uint8_t SpiDriver::spiReadStatus(uint8_t addr)
{
    uint8_t temp = addr | readBurst;
    uint8_t value;
    spiStart();
    digitalWrite(ssPin, LOW);
    while (digitalRead(misoPin));
    SPI.transfer(temp);
    value = SPI.transfer(0);
    digitalWrite(ssPin, HIGH);
    spiEnd();
    return value;
}