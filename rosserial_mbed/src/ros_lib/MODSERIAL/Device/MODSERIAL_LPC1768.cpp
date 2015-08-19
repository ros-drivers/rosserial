#ifdef TARGET_LPC1768
#include "MODSERIAL.h"


void MODSERIAL::setBase(void ) {
switch( _serial.index ) {
        case 0: _base = LPC_UART0; _IRQ = UART0_IRQn; break;
        case 1: _base = LPC_UART1; _IRQ = UART1_IRQn; break;
        case 2: _base = LPC_UART2; _IRQ = UART2_IRQn; break;
        case 3: _base = LPC_UART3; _IRQ = UART3_IRQn; break;
        default: _base = NULL; _IRQ = (IRQn_Type)NULL; break;
    }
}

void MODSERIAL::initDevice(void) {
    ((LPC_UART_TypeDef*)_base)->FCR = (1UL<<0) + (1UL<<1) + (1UL<<2);
    }

bool MODSERIAL::txIsBusy( void ) 
{ 
    return ( (((LPC_UART_TypeDef*)_base)->LSR & ( 1UL << 6 )) == 0 ) ? true : false; 
} 

#endif
