#pragma once

#include <cstdint>

/*
  Bare-metal SPI driver for MSP430G2553 using USCI_B0 (P1.5=SCLK, P1.6=MISO, P1.7=MOSI).
  CS/SS is a GPIO mask (P1.x bit mask) configurable via setSpiPin().
*/

class SpiDriver
{
public:
    SpiDriver();

    void spiStart();
    void spiEnd();

    uint8_t transferByte(uint8_t out);
    void spiWriteReg(uint8_t addr, uint8_t value);
    void spiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num);
    void spiStrobe(uint8_t strobe);
    uint8_t spiReadReg(uint8_t addr);
    void spiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num);
    uint8_t spiReadStatus(uint8_t addr);
    void setSs(bool value);

private:
    uint8_t ssMask; // P1.x mask for CS
};
