#ifndef UART_LISTENERS_H
#define UART_LISTENERS_H

namespace Uart {

class UartRxListener
{
public:
    virtual void onUartRx(unsigned char) = 0;
};

class UartTxListener
{
public:
    virtual void onUartTx() = 0;
};

} // namespace

#endif // UART_LISTENERS_H
