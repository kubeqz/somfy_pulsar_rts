#pragma once

#include <Arduino.h>

enum class ReceiverState
{
    IDLE,
    WAIT_FOR_PREAMBLE_END,
    DATA,
    FINISHED,
    DATA_ERROR,
    PREAMBLE_ERROR
};

constexpr unsigned MessageLength = 10;

struct Message
{
    union {
        uint8_t data[MessageLength];
        struct {
            uint8_t reserved;
            uint8_t crc;
            uint8_t counter[2];
            uint8_t buttonId;
            uint8_t remoteId0;
            uint8_t remoteId1;
            uint8_t const_data[3];
        } frame;
    };
};

class SomfyRtsReceiver
{
private:
    void convert_bit();
    void decrypt();
    void checkCrc();
    static void handleInterrupt();

    static bool m_wait;
    static bool m_received;
    static int m_interruptPin;
    static bool m_lastBit;
    static unsigned m_bitCount;

    static bool m_decodedBin[MessageLength * 8];
    uint8_t m_decodedSig[MessageLength];
    uint8_t m_decodedDec[MessageLength];
    uint8_t m_encryptedDec[MessageLength];

public:
    void init();
    void enableReceive();
    void disableReceive();

    void printDebug();
    ReceiverState getState();

    Message getMessage();

    bool receive();
    void clear();
};