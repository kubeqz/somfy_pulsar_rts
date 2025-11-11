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

class SomfyRtsReceiver
{
private:
    void convert_bit(void);
    void decrypt(void);
    void check_crc(void);
    void btn_repeat(void);
    static void handleInterrupt();

public:
    void init(void);
    void enableReceive(void);
    void disableReceive(void);

    void printDebug(void);
    ReceiverState getState(void);

    bool receive(void);
    int received_code(int addr, bool c);
    void clear_received(void);
};