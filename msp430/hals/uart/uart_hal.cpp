#include <hals/uart/uart_hal.h>
#include <msp430.h>

namespace Uart {

static UartRxListener* s_rxListener = nullptr;
static UartTxListener* s_txListener = nullptr;

extern "C" void usci0rx_isr(void) __attribute((interrupt(USCIAB0RX_VECTOR)));
extern "C" void usci0tx_isr(void) __attribute((interrupt(USCIAB0TX_VECTOR)));

void usci0rx_isr(void)
{
    if(s_rxListener != 0) {
        s_rxListener->onUartRx(UCA0RXBUF);
    }
}

void usci0tx_isr(void)
{
    if(s_txListener != 0) {
        s_txListener->onUartTx();
    }
}

void UartHal::initialise()
{
    s_rxListener = nullptr;
    s_txListener = nullptr;

    P1SEL  = BIT1 + BIT2;                     // set pin functions to UCA0RXD and UCA0TXD
    P1SEL2 = BIT1 + BIT2;
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 = 0x41;                           // 16MHz 9600
    UCA0BR1 = 0x03;                           // 16MHz 9600
    UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
}

void UartHal::setRxListener(UartRxListener& rxListener)
{
    s_rxListener = &rxListener;
}

void UartHal::setTxListener(UartTxListener& txListener)
{
    s_txListener = &txListener;
}

unsigned char UartHal::get()
{
    return UCA0RXBUF;
}

void UartHal::put(unsigned char c)
{
    UCA0TXBUF = c;
}

void UartHal::enableTxIrq(bool enable)
{
    if (enable) {
        IE2 |= UCA0TXIE;
    } else {
        IE2 &= ~UCA0TXIE;
    }
}

void UartHal::enableRxIrq(bool enable)
{
    if (enable) {
        IE2 |= UCA0RXIE;
    } else {
        IE2 &= ~UCA0RXIE;
    }
}

} // namespace
