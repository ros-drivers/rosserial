#ifdef TARGET_KL05Z
#include "MODSERIAL.h"

void MODSERIAL::setBase(void ) {
    _base = UART0;
    _IRQ  = UART0_IRQn;
}

void MODSERIAL::initDevice(void) {};

bool MODSERIAL::txIsBusy( void ) 
{ 
    return ( ((UARTLP_Type*)_base)->S1 & ( 1UL << 6 ) == 0 ) ? true : false; 
} 
#endif

