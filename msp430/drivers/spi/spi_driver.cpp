// MSP430G2553 bare-metal SPI driver for CC1101 (replaces Arduino SPI usage)
#include "spi_driver.h"
#include <msp430.h>

/*
 Notes:
 - Uses USCI_B0 (P1.5=SCLK, P1.6=MISO, P1.7=MOSI).
 - CS (SS) defaults to P1.4 (BIT4).
 - No Arduino APIs used.
*/

constexpr uint8_t writeBurst = 0x40; // write burst
constexpr uint8_t readSingle = 0x80; // read single
constexpr uint8_t readBurst = 0xC0;  // read burst

SpiDriver::SpiDriver()
    : ssMask(BIT4) // default CS = P1.4
{
    // Configure USCI_B0 pins function select later in spiStart().
    // Default SS is P1.4 (drive high when idle).
    ssMask = BIT4;
    // Ensure SS pin is output high
    P1DIR |= ssMask;
    P1OUT |= ssMask;
}

uint8_t SpiDriver::transferByte(uint8_t out)
{
    // Write TX buffer and wait for RX
    while (!(IFG2 & UCB0TXIFG)) {} // wait TX ready
    UCB0TXBUF = out;
    while (!(IFG2 & UCB0RXIFG)) {} // wait RX received
    return (uint8_t)UCB0RXBUF;
}

void SpiDriver::setSs(bool value)
{
    if (value) {
        P1OUT |= ssMask;
    } else {
        P1OUT &= ~ssMask;
    }
}

void SpiDriver::spiStart()
{
    // Put USCI_B0 into reset
    UCB0CTL1 |= UCSWRST;

    // Configure USCI_B0 as SPI master, 3-pin, MSB first, synchronous, SMCLK
    UCB0CTL0 = UCCKPH | UCMSB | UCMST | UCSYNC; // Mode: 3-pin SPI, master, MSB, clock phase = 1
    UCB0CTL1 = UCSSEL_2 | UCSWRST;              // SMCLK, hold in reset

    // Set bit rate (SMCLK / 2)
    UCB0BR0 = 0x00;
    UCB0BR1 = 0x00;

    // Select peripheral function for P1.5/P1.6/P1.7
    P1SEL |= BIT5 | BIT6 | BIT7;
    P1SEL2 |= BIT5 | BIT6 | BIT7;

    // SS pin high (idle)
    P1OUT |= ssMask;

    // Release USCI state machine
    UCB0CTL1 &= ~UCSWRST;
}

void SpiDriver::spiEnd()
{
    P1DIR |= ssMask;
}

void SpiDriver::spiWriteReg(uint8_t addr, uint8_t value)
{
    // Assert CS low
    P1OUT &= ~ssMask;
    __delay_cycles(10);

    // send addr then value (ignore returned bytes)
    transferByte(addr);
    transferByte(value);

    // Deassert CS high
    __delay_cycles(10);
    P1OUT |= ssMask;
}

void SpiDriver::spiWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num)
{
    uint8_t temp = addr | writeBurst;
    P1OUT &= ~ssMask;
    __delay_cycles(10);

    transferByte(temp);
    for (uint8_t i = 0; i < num; ++i)
    {
        transferByte(buffer[i]);
    }

    __delay_cycles(10);
    P1OUT |= ssMask;
}

void SpiDriver::spiStrobe(uint8_t strobe)
{
    P1OUT &= ~ssMask;
    __delay_cycles(10);

    transferByte(strobe);

    __delay_cycles(10);
    P1OUT |= ssMask;
}

uint8_t SpiDriver::spiReadReg(uint8_t addr)
{
    uint8_t temp = addr | readSingle;
    uint8_t value = 0;

    P1OUT &= ~ssMask;
    __delay_cycles(10);

    transferByte(temp);         // send address, receive status byte (ignored here)
    value = transferByte(0x00); // send dummy to clock in register value

    __delay_cycles(10);
    P1OUT |= ssMask;
    return value;
}

void SpiDriver::spiReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t num)
{
    uint8_t temp = addr | readBurst;

    P1OUT &= ~ssMask;
    __delay_cycles(10);

    transferByte(temp); // send address, receive status

    for (uint8_t i = 0; i < num; ++i)
    {
        buffer[i] = transferByte(0x00);
    }

    __delay_cycles(10);
    P1OUT |= ssMask;
}

uint8_t SpiDriver::spiReadStatus(uint8_t addr)
{
    // Status reads use readBurst in original driver
    uint8_t temp = addr | readBurst;
    uint8_t status = 0;

    P1OUT &= ~ssMask;
    __delay_cycles(10);

    transferByte(temp);
    status = transferByte(0x00);

    __delay_cycles(10);
    P1OUT |= ssMask;
    return status;
}
