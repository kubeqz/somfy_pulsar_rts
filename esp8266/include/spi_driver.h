#pragma once

#include <Arduino.h>
#include <SPI.h>

class SpiDriver
{
private:
    uint8_t sckPin;
    uint8_t misoPin;
    uint8_t mosiPin;
    uint8_t ssPin;

public:
    SpiDriver();

    void setSpiPin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t ss);
    void setSpiDefault();
    void spiStart();
    void spiEnd();
    void spiWriteReg(uint8_t addr, uint8_t value);
    void spiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num);
    void spiStrobe(uint8_t strobe);
    uint8_t spiReadReg(uint8_t addr);
    void spiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num);
    uint8_t spiReadStatus(uint8_t addr);

    uint8_t getSckPin() const { return sckPin; }
    uint8_t getMisoPin() const { return misoPin; }
    uint8_t getMosiPin() const { return mosiPin; }
    uint8_t getSsPin() const { return ssPin; }
};