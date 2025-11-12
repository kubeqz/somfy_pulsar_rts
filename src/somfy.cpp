#include "somfy.h"
#include "cc1101.h"

#include <Arduino.h>

constexpr unsigned PULSE_TIME_US_MIN = 500;
constexpr unsigned PULSE_TIME_US_MAX = 800;
constexpr unsigned PULSE_TIME_US = 650;

constexpr unsigned DEBUG_PIN = 16; // GPIO16 (D0 on Wemos D1 mini)

#if defined(ESP8266)
#define RECEIVE_ATTR ICACHE_RAM_ATTR
#elif defined(ESP32)
#define RECEIVE_ATTR IRAM_ATTR
#else
#define RECEIVE_ATTR
#endif

#define RX_PIN   4 //CC1101 : GDO2
#define TX_PIN   5 //CC1101 : GDO0
#define SCK_PIN  14 //CC1101 : SCK
#define MISO_PIN 12 //CC1101 : MISO
#define MOSI_PIN 13 //CC1101 : MOSI
#define CSN_PIN  15  //CC1101 : CSN

bool SomfyRtsReceiver::m_wait = 0;
bool SomfyRtsReceiver::m_received = 0;
int SomfyRtsReceiver::m_interruptPin = 0;
bool SomfyRtsReceiver::m_lastBit = 0;
unsigned SomfyRtsReceiver::m_bitCount = 0;
bool SomfyRtsReceiver::m_decodedBin[MessageLength * 8];

ReceiverState m_receiverState = ReceiverState::IDLE;
CC1101 cc1101;

void SomfyRtsReceiver::init()
{
    cc1101.setGdo(TX_PIN, RX_PIN);
    cc1101.setSpiPin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

    cc1101.init();
    cc1101.setMhz(433.42);
}

void SomfyRtsReceiver::enableReceive()
{
    pinMode(RX_PIN, INPUT);
    cc1101.setRx();
    attachInterrupt(digitalPinToInterrupt(RX_PIN), handleInterrupt, CHANGE);
}

void SomfyRtsReceiver::disableReceive()
{
    detachInterrupt(m_interruptPin);
}

ReceiverState SomfyRtsReceiver::getState()
{
    return m_receiverState;
}

void SomfyRtsReceiver::printDebug()
{
    Serial.printf("State: %u, bit cnt: %u\r\n", static_cast<unsigned>(m_receiverState), m_bitCount);

    for (unsigned bit = 0; bit < m_bitCount; ++bit) {
        Serial.printf("bit[%d] = %d\r\n", bit, m_decodedBin[bit]);
    }
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
        Serial.println("CRC failed");
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
    attachInterrupt(m_interruptPin, handleInterrupt, CHANGE);
}

void RECEIVE_ATTR SomfyRtsReceiver::handleInterrupt()
{
    long time = micros();

    static unsigned long lastTime = 0;
    const unsigned int duration = time - lastTime;
    static unsigned long resetTime = 0;

    if (millis() - resetTime > 100)
    {
        m_bitCount = 0;
        m_lastBit = 0;
        m_receiverState = ReceiverState::WAIT_FOR_PREAMBLE_END;
        m_wait = 0;

        lastTime = time;
        resetTime = millis();
        return;
    }

    if ((m_receiverState == ReceiverState::PREAMBLE_ERROR) ||
        (m_receiverState == ReceiverState::DATA_ERROR))
    {
        // skip processing until enough time pass or manual receive reset
        return;
    }

    if (duration > 100)
    {
        if (m_receiverState == ReceiverState::WAIT_FOR_PREAMBLE_END)
        {
            if ( (duration > 2400) && (duration < 2700))
            {
                m_receiverState = ReceiverState::WAIT_FOR_PREAMBLE_END;
            }
            else if ((duration > 4700) && (duration < 5000))
            {
                // SW sync detected, this means end of preamble

                m_receiverState = ReceiverState::DATA;
                m_lastBit = 1;
                time += PULSE_TIME_US;
            }
            else
            {
                // unexpected time between edges
                m_receiverState = ReceiverState::PREAMBLE_ERROR;
                detachInterrupt(m_interruptPin);
            }
        }
        else if (m_receiverState == ReceiverState::DATA)
        {
            if ((duration > (2 * PULSE_TIME_US_MIN)) && (duration < (2 * PULSE_TIME_US_MAX)))
            {
                if (m_lastBit == 1)
                {
                    m_lastBit = 0;   // falling edge -> 0
                    m_decodedBin[m_bitCount] = 0;
                    m_bitCount++;
                }
                else
                {
                    m_lastBit = 1;   // rising edge -> 1
                    m_decodedBin[m_bitCount] = 1;
                    m_bitCount++;
                }
            }
            else if (duration > PULSE_TIME_US_MIN && duration < PULSE_TIME_US_MAX)
            {
                if (m_wait == 1)
                {
                    m_wait = 0;
                }
                else
                {
                    m_wait = 1;
                    m_decodedBin[m_bitCount] = m_lastBit;
                    m_bitCount++;
                }
            }
            else
            {
                m_receiverState = ReceiverState::DATA_ERROR;
                detachInterrupt(m_interruptPin);
            }
        }

        if (m_bitCount == 80)
        {
            m_receiverState = ReceiverState::FINISHED;
            detachInterrupt(m_interruptPin);
        }
    }

    lastTime = time;
    resetTime = millis();
}

Message SomfyRtsReceiver::getMessage()
{
    Message message;
    memcpy(message.data, m_encryptedDec, sizeof(message));
    return message;
}