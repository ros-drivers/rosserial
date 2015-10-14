#ifdef TARGET_LPC11U24
#include "MODSERIAL.h"

void MODSERIAL::setBase(void ) {
    _base = LPC_USART;
    _IRQ = UART_IRQn;
}

void MODSERIAL::initDevice(void) {
    ((LPC_USART_Type*)_base)->FCR = (1UL<<0) + (1UL<<1) + (1UL<<2);
    }

bool MODSERIAL::txIsBusy( void ) 
{ 
    return ( (((LPC_USART_Type*)_base)->LSR & ( 1UL << 6 )) == 0 ) ? true : false; 
} 
#endif
