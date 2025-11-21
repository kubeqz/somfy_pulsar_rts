#ifndef UART_HAL_H
#define UART_HAL_H

#include "uart_listeners.h"

namespace Uart {

class UartHal
{
public:
    void initialise();

    unsigned char get();
    void put(unsigned char c);

    void setRxListener(UartRxListener& listener);
    void setTxListener(UartTxListener& listener);

    void enableTxIrq(bool enable);
    void enableRxIrq(bool enable);
};

} // namespace

#endif // UART_HAL_H
