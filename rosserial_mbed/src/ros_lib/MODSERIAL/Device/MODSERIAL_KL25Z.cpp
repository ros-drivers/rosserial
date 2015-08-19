#ifdef TARGET_KL25Z
#include "MODSERIAL.h"

void MODSERIAL::setBase(void ) {
switch( _serial.index ) {
        case 0: _base = UART0; _IRQ = UART0_IRQn; break;
        case 1: _base = UART1; _IRQ = UART1_IRQn; break;
        case 2: _base = UART2; _IRQ = UART2_IRQn; break;
        default: _base = NULL; _IRQ = (IRQn_Type)NULL; break;
    }
}

void MODSERIAL::initDevice(void) {};

bool MODSERIAL::txIsBusy( void ) 
{ 
    return ( (((UART_Type*)_base)->S1 & ( 1UL << 6 )) == 0 ) ? true : false; 
} 
#endif

