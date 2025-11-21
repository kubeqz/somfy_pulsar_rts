#include "drivers/somfy/somfy.h"
#include "drivers/cc1101/cc1101.h"
#include "drivers/spi/spi_driver.h"

#include <msp430.h>
#include <cstdint>
#include <cstring>

/* --- Pin mapping (MSP430G2553) ---
   SPI (USCI_B0): P1.5 = SCK, P1.6 = MISO, P1.7 = MOSI
   CS (SS) default: P1.4 (BIT4)
   GDO pins: P2.1 (GDO2 / RX)
*/

constexpr uint8_t GDO2_PIN_MASK = BIT1; // P2.1

constexpr unsigned PULSE_TIME_US_MIN = 500;
constexpr unsigned PULSE_TIME_US_MAX = 800;
constexpr unsigned PULSE_TIME_US = 650;

bool SomfyRtsReceiver::m_wait = 0;
bool SomfyRtsReceiver::m_received = 0;
int SomfyRtsReceiver::m_interruptPin = 0;
bool SomfyRtsReceiver::m_lastBit = 0;
unsigned SomfyRtsReceiver::m_bitCount = 0;
bool SomfyRtsReceiver::m_decodedBin[MessageLength * 8];

// static CC1101 instance used by somfy
extern CC1101 cc1101; // defined in cc1101.cpp

static volatile uint32_t g_millis = 0;

static void timer_init()
{
    // Configure Timer_A0 CCR0 for 1ms ticks: SMCLK / 1000
    TA0CCTL0 = CCIE;            // CCR0 interrupt enabled
    TA0CCR0 = 8000 - 1;         // assuming SMCLK ~= 8MHz -> 1 ms
    TA0CTL = TASSEL_2 | MC_1 | TACLR; // SMCLK, up mode

    TA1CCTL0 = 0;
    // SMCLK, divide clock /8 (1 tick is 1 us), count up to 0xFFFF
    TA1CTL = TASSEL_2 | MC_2 | TACLR | ID_3;
}

extern "C" __attribute__((interrupt(TIMER0_A0_VECTOR))) void Timer_A0_CCR0_ISR()
{
    ++g_millis;
    TA0CCTL0 &= ~CCIFG;
}

static inline uint32_t millis_ms()
{
    return g_millis;
}

static inline uint32_t micros_us()
{
    return TA1R;
}

static inline void micros_reset()
{
    TA1R = 0;
}

/* --- Port2 interrupt glue for GDO2 (RX) ---
   We'll use P2.1 as RX (GDO2). ISR calls SomfyRtsReceiver::handleInterrupt()
*/
static inline void attach_port2_change(uint8_t mask)
{
    P2IFG &= ~mask;      // clear any pending flags
    P2IE |= mask;        // enable interrupt
    // Use initial edge = rising (arbitrary); handler will toggle to detect change
    P2IES &= ~mask;      // interrupt on low->high first
}

static inline void detach_port2(uint8_t mask)
{
    P2IE &= ~mask;
    P2IFG &= ~mask;
}

/* Port2 ISR wrapper */
extern "C" __attribute__((interrupt(PORT2_VECTOR))) void Port2_ISR()
{
    // check which pin triggered; only care about GDO2 mask
    if (P2IFG & GDO2_PIN_MASK)
    {
        // call handler
        SomfyRtsReceiver::handleInterrupt();
        // toggle edge so CHANGE is detected
        P2IES ^= GDO2_PIN_MASK;
        // clear flag
        P2IFG &= ~GDO2_PIN_MASK;
    }
}

ReceiverState m_receiverState = ReceiverState::IDLE; // fallback if not in class
CC1101 cc1101; // ensure global instantiation

void SomfyRtsReceiver::init()
{
    timer_init();

    cc1101.init();
    cc1101.setMhz(433.42f);
}

void SomfyRtsReceiver::enableReceive()
{
    // configure P2.1 as input with pull-down disabled
    P2DIR &= ~GDO2_PIN_MASK; // input
    P2REN &= ~GDO2_PIN_MASK; // no pull resistor

    cc1101.setRx();

    P2DIR |= BIT2;

    // clear flags and enable change interrupts
    attach_port2_change(GDO2_PIN_MASK);
    // ensure CPU interrupts enabled
    __enable_interrupt();
}

void SomfyRtsReceiver::disableReceive()
{
    detach_port2(GDO2_PIN_MASK);
}

ReceiverState SomfyRtsReceiver::getState()
{
    return m_receiverState;
}

void SomfyRtsReceiver::printDebug()
{
    // no Serial on bare-metal; function kept as stub
}

bool SomfyRtsReceiver::receive()
{
    if (m_bitCount == 80)
    {
        convert_bit();
        decrypt();
        checkCrc();
    }
    return m_received;
}

void SomfyRtsReceiver::convert_bit()
{
    int c = 0;
    for (unsigned i = 0; i < MessageLength; i++)
    {
        int calc = 0x80;
        for (int i2 = 0; i2 < 8; i2++)
        {
            if (m_decodedBin[c] == 1)
            {
                m_decodedSig[i] += calc;
            }
            calc >>= 1;
            c++;
        }
    }
    for (unsigned i = 0; i < MessageLength; i++)
    {
        m_decodedDec[i] = m_decodedSig[i];
    }
}

void SomfyRtsReceiver::decrypt()
{
    m_encryptedDec[0] = m_decodedDec[0];
    for (unsigned i = 0; i < (MessageLength - 1); i++)
    {
        m_decodedSig[i] ^= m_decodedSig[i + 1];
    }
    for (unsigned i = 1; i < MessageLength; i++)
    {
        m_encryptedDec[i] = m_decodedSig[i - 1];
    }
}

void SomfyRtsReceiver::checkCrc()
{
    unsigned crc = 0;
    unsigned checkcrc = m_encryptedDec[1];
    m_encryptedDec[0] = 163;
    m_encryptedDec[1] = 0;
    for (unsigned i = 0; i < MessageLength; i++)
    {
        crc = crc ^ m_encryptedDec[i] ^ (m_encryptedDec[i] >> 4);
    }
    crc &= 0b1111;

    m_encryptedDec[0] += 1;
    m_encryptedDec[1] = checkcrc;

    if (checkcrc - 240 == crc)
    {
        m_bitCount = 0;
        m_received = 1;
    }
    else
    {
        m_received = 0;
    }
}

void SomfyRtsReceiver::clear()
{
    m_received = 0;
    m_bitCount = 0;
    m_lastBit = 0;
    m_receiverState = ReceiverState::IDLE;
    m_wait = 0;

    for (unsigned i = 0; i < MessageLength; i++)
    {
        m_decodedSig[i] = 0;
        m_decodedDec[i] = 0;
        m_encryptedDec[i] = 0;
    }

    // re-enable interrupts for RX
    attach_port2_change(GDO2_PIN_MASK);
}

void SomfyRtsReceiver::handleInterrupt()
{
    static uint32_t additionalTime = 0;
    static uint32_t resetTime = 0;

    uint32_t usTimeBetweenEdges = micros_us() - additionalTime;
    additionalTime = 0;

    // start measuring time to the next edge
    micros_reset();

    if ((g_millis - resetTime) > 100)
    {
        m_bitCount = 0;
        m_lastBit = 0;
        m_receiverState = ReceiverState::WAIT_FOR_PREAMBLE_END;
        m_wait = 0;

        resetTime = millis_ms();
        return;
    }

    resetTime = g_millis;

    if ((m_receiverState == ReceiverState::PREAMBLE_ERROR) ||
        (m_receiverState == ReceiverState::DATA_ERROR))
    {
        // skip processing until enough time pass or manual receive reset
        return;
    }

    if (usTimeBetweenEdges > 100)
    {
        if (m_receiverState == ReceiverState::WAIT_FOR_PREAMBLE_END)
        {
            if ((usTimeBetweenEdges > 2400) && (usTimeBetweenEdges < 2700))
            {
                m_receiverState = ReceiverState::WAIT_FOR_PREAMBLE_END;
            }
            else if ((usTimeBetweenEdges > 4700) && (usTimeBetweenEdges < 5000))
            {
                // SW sync detected, this means end of preamble
                m_receiverState = ReceiverState::DATA;
                m_lastBit = 1;
                // shift time by PULSE_TIME_US as we're in the middle of symbol
                additionalTime = PULSE_TIME_US;
            }
            else
            {
                // unexpected time between edges
                m_receiverState = ReceiverState::PREAMBLE_ERROR;
                detach_port2(GDO2_PIN_MASK);
            }
        }
        else if (m_receiverState == ReceiverState::DATA)
        {
            if ((usTimeBetweenEdges > (2 * PULSE_TIME_US_MIN)) && (usTimeBetweenEdges < (2 * PULSE_TIME_US_MAX)))
            {
                if (m_lastBit == 1)
                {
                    m_lastBit = 0; // falling edge -> 0
                    m_decodedBin[m_bitCount] = 0;
                    ++m_bitCount;
                }
                else
                {
                    m_lastBit = 1; // rising edge -> 1
                    m_decodedBin[m_bitCount] = 1;
                    ++m_bitCount;
                }
            }
            else if ((usTimeBetweenEdges > PULSE_TIME_US_MIN) && (usTimeBetweenEdges < PULSE_TIME_US_MAX))
            {
                if (m_wait == 1)
                {
                    m_wait = 0;
                }
                else
                {
                    m_wait = 1;
                    m_decodedBin[m_bitCount] = m_lastBit;
                    ++m_bitCount;
                }
            }
            else
            {
                m_receiverState = ReceiverState::DATA_ERROR;
                detach_port2(GDO2_PIN_MASK);
            }
        }

        if (m_bitCount == 80)
        {
            m_receiverState = ReceiverState::FINISHED;
            detach_port2(GDO2_PIN_MASK);
        }
    }
}

Message SomfyRtsReceiver::getMessage()
{
    Message message;
    memcpy(message.data, m_encryptedDec, sizeof(message.data));
    return message;
}
