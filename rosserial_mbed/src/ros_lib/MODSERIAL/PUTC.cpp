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

int
MODSERIAL::__putc(int c, bool block) {
    
    // If no buffer is in use fall back to standard TX FIFO usage.
    // Note, we must block in this case and ignore bool "block" 
    // so as to maintain compat with Mbed Serial.
    if (buffer[TxIrq] == (char *)NULL || buffer_size[TxIrq] == 0) {
        while (! MODSERIAL_THR_HAS_SPACE) ; // Wait for space in the TX FIFO.
        _THR = (uint32_t)c;
        return 0;
    }
    
    if ( MODSERIAL_THR_HAS_SPACE && MODSERIAL_TX_BUFFER_EMPTY && dmaSendChannel == -1 ) {
        _THR = (uint32_t)c;
    }
    else {
        if (block) {
            uint32_t ier = _IER; _IER = 1;
            while ( MODSERIAL_TX_BUFFER_FULL ) {  // Blocks!
                // If putc() is called from an ISR then we are stuffed
                // because in an ISR no bytes from the TX buffer will 
                // get transferred to teh TX FIFOs while we block here.
                // So, to work around this, instead of sitting in a 
                // loop waiting for space in the TX buffer (which will
                // never happen in IRQ context), check to see if the
                // TX FIFO has space available to move bytes from the
                // TX buffer to TX FIFO to make space. The easiest way
                // to do this is to poll the isr_tx() function while we
                // are blocking.
                isr_tx(false);
            }
            _IER = ier;
        }
        else if( MODSERIAL_TX_BUFFER_FULL ) {
            buffer_overflow[TxIrq] = c; // Oh dear, no room in buffer.
            _isr[TxOvIrq].call(&this->callbackInfo);
            return -1;
        }
        _IER &= ~2;
        buffer[TxIrq][buffer_in[TxIrq]] = c;
        __disable_irq();
        buffer_count[TxIrq]++;
        __enable_irq();
        buffer_in[TxIrq]++;
        if (buffer_in[TxIrq] >= buffer_size[TxIrq]) {
            buffer_in[TxIrq] = 0;
        }            
        _IER |= 2;        
    }
      
    return 0;
}

}; // namespace AjK ends
