/* ========================================
 *
 * The following firmware was developed by Chuck Harrison
 * This work is licensed under a Creative Commons Attribution 3.0 Unported License.
 * 
 * http://creativecommons.org/licenses/by/3.0/deed.en_US
 * 
 * You are free to:
 * -To Share — to copy, distribute and transmit the work 
 * -To Remix — to adapt the work 
 * -To make commercial use of the work
 *
 * ========================================
 */

#ifndef ROS_UARTS_H_
#define ROS_UARTS_H_

#include "HardwareSerial.h"

extern "C" {
void     UART_SpiUartWriteTxData(uint32_t value) ;
uint32_t UART_UartGetByte(void) ;

void    UART_Start() ; 
}

class Uart : public HardwareSerial
{
  public:
    void begin(unsigned long baud) { UART_Start(); } ; // TBD: set baud rate
    int read(void) { 
      uint32_t data =UART_UartGetByte();
      if (data&0xffffff00) {
        return -1;
      } else {
        return data;
      }
    } ;
    size_t write(uint8_t data) { UART_SpiUartWriteTxData((uint32_t)data); return 1; } ;
} ;

extern Uart Uart0;

#endif

