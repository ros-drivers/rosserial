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

    #define MODSERIAL_FCR  0x08
    #define _FCR    *((char *)_base+MODSERIAL_FCR)
    
    #define MODSERIAL_FIFO_ENABLE   1
#define MODSERIAL_FIFO_RX_RESET 2
#define MODSERIAL_FIFO_TX_RESET 4


namespace AjK {

void
MODSERIAL::init( int txSize, int rxSize, PinName rx )
{
    
    NVIC_DisableIRQ(_IRQ);
    setBase();

    callbackInfo.setSerial(this);

    
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
    

    initDevice();

    //_FCR = MODSERIAL_FIFO_ENABLE | MODSERIAL_FIFO_RX_RESET | MODSERIAL_FIFO_TX_RESET;
    
    auto_detect_char = 0;
    
    NVIC_EnableIRQ(_IRQ);
}

}; // namespace AjK ends

