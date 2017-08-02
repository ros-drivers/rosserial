#ifndef MKL82Z_UART_H
#define MKL82Z_UART_H

#include "fsl_lpuart.h"
#include "fsl_pit.h"


#define MKL82Z_LPUART LPUART0
#define MKL82Z_LPUART_IRQn LPUART0_IRQn
#define MKL82Z_LPUART_IRQHandler LPUART0_IRQHandler
#define MKL82Z_PIT0_IRQHandler PIT0_IRQHandler
#define MKL82Z_CLK_SEL kCLOCK_PllFllSelClk
#define MKL82Z_PIT_SRC_CLK CLOCK_GetFreq(kCLOCK_BusClk)
#define MKL82Z_IRQ_ID PIT0_IRQn
#define UART_RX_BUFF_LEN 256
#define UART_TX_BUFF_LEN 256

extern volatile uint16_t Rx_UART_HEAD;
extern volatile uint16_t Rx_UART_TAIL;
extern volatile uint16_t Tx_UART_HEAD;
extern volatile uint16_t Tx_UART_TAIL;

void MKL82Z_PIT0_IRQHandler();
void MKL82Z_LPUART_IRQHandler();
void MKL82Z_UART_init();
void MKL82Z_PIT_init();
int16_t MKL82Z_UART_recbyte();
int MKL82Z_UART_write_byte(uint8_t data);
uint32_t MKL82Z_time();

#endif /* MKL82Z_UART_H */
