#ifdef TARGET_KPSDK_MCUS
#include "MODSERIAL.h"

void MODSERIAL::setBase(void ) {
switch( _serial.index ) {
        case 0: _base = UART0; _IRQ = UART0_RX_TX_IRQn; break;
        case 1: _base = UART1; _IRQ = UART1_RX_TX_IRQn; break;
        case 2: _base = UART2; _IRQ = UART2_RX_TX_IRQn; break;
        #ifdef TARGET_K64F
        case 3: _base = UART3; _IRQ = UART3_RX_TX_IRQn; break;
        case 4: _base = UART4; _IRQ = UART4_RX_TX_IRQn; break;
        #endif
        default: _base = NULL; _IRQ = (IRQn_Type)NULL; break;
    }
}

void MODSERIAL::initDevice(void) {};

bool MODSERIAL::txIsBusy( void ) 
{ 
    return ( ((UART_Type*)_base)->S1 & ( 1UL << 6 ) == 0 ) ? true : false; 
} 
#endif