/*
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

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
