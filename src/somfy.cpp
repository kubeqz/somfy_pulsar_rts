#include "somfy.h"
#include "cc1101.h"

#include <EEPROM.h>
#include <Arduino.h>

constexpr unsigned PULSE_TIME_US_MIN = 500;
constexpr unsigned PULSE_TIME_US_MAX = 800;
constexpr unsigned PULSE_TIME_US = 650;

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

int RxPin = RX_PIN;

bool wait = 0;
bool received = 0;
int crc;
int checkcrc;
static unsigned long resetTime = 0;

static int interruptPin = 0;
unsigned bit_count = 0;
bool last_bit;
bool decoded_bin[80];
uint8_t decoded_sig[11];
uint8_t decoded_dec[10];
uint8_t encrypted_dec[10];

ReceiverState receiverState = ReceiverState::IDLE;
CC1101 cc1101;

void SomfyRtsReceiver::init(void)
{
    cc1101.setGdo(TX_PIN, RX_PIN);
    cc1101.setSpiPin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

    cc1101.init();
    cc1101.setMhz(433.42);
}

void SomfyRtsReceiver::enableReceive(void)
{
    RxPin = RX_PIN;
    pinMode(RxPin, INPUT);
    RxPin = digitalPinToInterrupt(RxPin);
    cc1101.setRx();
    attachInterrupt(RxPin, handleInterrupt, CHANGE);
}

void SomfyRtsReceiver::disableReceive(void)
{
    detachInterrupt(interruptPin);
}

ReceiverState SomfyRtsReceiver::getState(void)
{
    return receiverState;
}

void SomfyRtsReceiver::printDebug(void)
{
    Serial.printf("State: %u, bit cnt: %u\r\n", static_cast<unsigned>(receiverState), bit_count);

    for (unsigned bit = 0; bit < bit_count; ++bit) {
        Serial.printf("bit[%d] = %d\r\n", bit, decoded_bin[bit]);
    }
}

bool SomfyRtsReceiver::receive(void)
{
    if (bit_count == 80)
    {
        convert_bit();
        decrypt();
        check_crc();
    }

    return received;
}

void SomfyRtsReceiver::convert_bit(void)
{
    int c = 0;
    for (int i = 0; i < 10; i++)
    {
        int calc = 128;
        for (int i2 = 0; i2 < 8; i2++)
        {
            if (decoded_bin[c] == 1)
            {
                decoded_sig[i] += calc;
            }
            calc /= 2;
            c++;
        }
    }
    for (int i = 0; i < 10; i++)
    {
        decoded_dec[i] = decoded_sig[i];
    }
}

void SomfyRtsReceiver::decrypt(void)
{
    encrypted_dec[0] = decoded_dec[0];
    for (int i = 0; i < 10; i++)
    {
        decoded_sig[i] ^= decoded_sig[i + 1];
    }
    for (int i = 1; i < 10; i++)
    {
        encrypted_dec[i] = decoded_sig[i - 1];
    }
}

void SomfyRtsReceiver::check_crc(void)
{
    crc = 0;
    checkcrc = encrypted_dec[1];
    encrypted_dec[0] = 163;
    encrypted_dec[1] = 0;
    for (int i = 0; i < 10; i++)
    {
        crc = crc ^ encrypted_dec[i] ^ (encrypted_dec[i] >> 4);
    }
    crc &= 0b1111;

    encrypted_dec[0] += 1;
    encrypted_dec[1] = checkcrc;

    if (checkcrc - 240 == crc)
    {
        bit_count = 0;
        received = 1;
    }
    else
    {
        received = 0;
    }
}

int SomfyRtsReceiver::received_code(int addr, bool c)
{
    if (c == 0)
    {
        return (decoded_dec[addr]);
    }
    else
    {
        return (encrypted_dec[addr]);
    }
}

void SomfyRtsReceiver::clear_received(void)
{
    received = 0;
    bit_count = 0;
    last_bit = 0;
    receiverState = ReceiverState::IDLE;
    wait = 0;

    for (int i = 0; i < 10; i++)
    {
        decoded_sig[i] = 0;
        decoded_dec[i] = 0;
        encrypted_dec[i] = 0;
    }
    attachInterrupt(interruptPin, handleInterrupt, CHANGE);
}

void RECEIVE_ATTR SomfyRtsReceiver::handleInterrupt()
{
    static unsigned long lastTime = 0;
    long time = micros();
    const unsigned int duration = time - lastTime;

    if (millis() - resetTime > 100)
    {
        Serial.printf("rst|");
        bit_count = 0;
        last_bit = 0;
        receiverState = ReceiverState::WAIT_FOR_PREAMBLE_END;
        wait = 0;

        lastTime = time;
        resetTime = millis();
        return;
    }

    if ((receiverState == ReceiverState::PREAMBLE_ERROR) ||
        (receiverState == ReceiverState::DATA_ERROR))
    {
        // skip processing until enough time pass or manual receive reset
        return;
    }

    if (duration > 100)
    {
        if (receiverState == ReceiverState::WAIT_FOR_PREAMBLE_END)
        {
            if ( (duration > 2400) && (duration < 2700))
            {
                receiverState = ReceiverState::WAIT_FOR_PREAMBLE_END;
            }
            else if ((duration > 4700) && (duration < 5000))
            {
                // SW sync detected, this means end of preamble

                receiverState = ReceiverState::DATA;
                last_bit = 1;
                time += PULSE_TIME_US;
            }
            else
            {
                // unexpected time between edges
                receiverState = ReceiverState::PREAMBLE_ERROR;
                detachInterrupt(interruptPin);
            }
        }
        else if (receiverState == ReceiverState::DATA)
        {
            Serial.printf("%d\r\n", duration);

            if ((duration > (2 * PULSE_TIME_US_MIN)) && (duration < (2 * PULSE_TIME_US_MAX)))
            {
                if (last_bit == 1)
                {
                    last_bit = 0;   // falling edge -> 0
                    decoded_bin[bit_count] = 0;
                    bit_count++;
                }
                else
                {
                    last_bit = 1;   // rising edge -> 1
                    decoded_bin[bit_count] = 1;
                    bit_count++;
                }
            }
            else if (duration > PULSE_TIME_US_MIN && duration < PULSE_TIME_US_MAX)
            {
                if (wait == 1)
                {
                    wait = 0;
                }
                else
                {
                    wait = 1;
                    decoded_bin[bit_count] = last_bit;
                    bit_count++;
                }
            }
            else
            {
                receiverState = ReceiverState::DATA_ERROR;
                detachInterrupt(interruptPin);
            }
        }

        if (bit_count == 80)
        {
            receiverState = ReceiverState::FINISHED;
            detachInterrupt(interruptPin);
        }
    }

    lastTime = time;
    resetTime = millis();
}