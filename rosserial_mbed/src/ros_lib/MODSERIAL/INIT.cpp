/*
    Copyright (c) 2010 Andy Kirkham
 
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
 
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
 
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "MODSERIAL.h"
#include "MACROS.h"


namespace AjK {

void
MODSERIAL::init( int txSize, int rxSize, PinName rx )
{
    disableIrq();
    
    callbackInfo.setSerial(this);

#ifdef __LPC11UXX_H__

    _base = LPC_USART;
    
#elif defined MKL25Z4_H_
    switch( _serial.index ) {
        case 0: _base = UART0; break;
        case 1: _base = UART1; break;
        case 2: _base = UART2; break;
        default: _base = NULL;      break;
    }
#else    
    switch( _serial.index ) {
        case 0: _base = LPC_UART0; break;
        case 1: _base = LPC_UART1; break;
        case 2: _base = LPC_UART2; break;
        case 3: _base = LPC_UART3; break;
        default: _base = NULL;      break;
    }
#endif
    
    dmaSendChannel  = -1;
    moddma_p        = (void *)NULL;
    
    if ( _base != NULL ) {
        buffer_size[RxIrq]     = rxSize;
        buffer[RxIrq]          = rxSize > 0 ? (char *)malloc(buffer_size[RxIrq]) : (char *)NULL;
        buffer_in[RxIrq]       = 0;
        buffer_out[RxIrq]      = 0;
        buffer_count[RxIrq]    = 0;
        buffer_overflow[RxIrq] = 0;
        Serial::attach( this, &MODSERIAL::isr_rx, Serial::RxIrq );        
        
        buffer_size[TxIrq]     = txSize;
        buffer[TxIrq]          = txSize > 0 ? (char *)malloc(buffer_size[TxIrq]) : (char *)NULL;
        buffer_in[TxIrq]       = 0;
        buffer_out[TxIrq]      = 0;
        buffer_count[TxIrq]    = 0;
        buffer_overflow[TxIrq] = 0;
        Serial::attach( this, &MODSERIAL::isr_tx, Serial::TxIrq );
    }
    else {
        error("MODSERIAL must have a defined UART to function.");
    }
    
    _FCR = MODSERIAL_FIFO_ENABLE | MODSERIAL_FIFO_RX_RESET | MODSERIAL_FIFO_TX_RESET;
    
    auto_detect_char = 0;
    
    enableIrq();
}

}; // namespace AjK ends
