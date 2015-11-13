#ifdef TARGET_PAC_F401RB
#include "MODSERIAL.h"

void MODSERIAL::setBase(void ) {
switch( _serial.index ) {
        case 0: _base = USART1; _IRQ = USART1_IRQn; break;
        case 1: _base = USART2; _IRQ = USART2_IRQn; break;
        case 2: _base = USART6; _IRQ = USART6_IRQn; break;
        default: _base = NULL; _IRQ = (IRQn_Type)NULL; break;
    }
}

void MODSERIAL::initDevice(void) {};

bool MODSERIAL::txIsBusy( void ) 
{ 
    return ( (((USART_TypeDef*)_base)->SR & ( 1UL << 6 )) == 0 ) ? true : false; 
} 

#endif
