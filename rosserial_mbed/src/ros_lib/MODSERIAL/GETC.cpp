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
MODSERIAL::__getc(bool block)
{
    // If no buffer is in use fall back to standard RX FIFO usage.
    // Note, we must block in this case and ignore bool "block" 
    // so as to maintain compat with Mbed Serial.
    if (buffer_size[RxIrq] == 0 || buffer[RxIrq] == (char *)NULL) {
        while(! MODSERIAL_RBR_HAS_DATA ) ;
        return (int)(_RBR & 0xFF);
    }

    if (block) { while ( MODSERIAL_RX_BUFFER_EMPTY ) ; } // Blocks.
    else if ( MODSERIAL_RX_BUFFER_EMPTY ) return -1;
    
    int c = buffer[RxIrq][buffer_out[RxIrq]];
    buffer_out[RxIrq]++;
    if (buffer_out[RxIrq] >= buffer_size[RxIrq]) {
        buffer_out[RxIrq] = 0;
    }
    
    // If we have made space in the RX Buffer then copy over
    // any characters in the RX FIFO that my reside there.
    // Temporarily disable the RX IRQ so that we do not re-enter 
    // it under interrupts.
    if ( ! MODSERIAL_RX_BUFFER_FULL ) {
        uint32_t ier = _IER;
        _IER &= ~(1UL << 0);
        isr_rx();    
        _IER = ier;
    }
    
    __disable_irq();
    buffer_count[RxIrq]--;   
    __enable_irq();
    return c;
}

}; // namespace AjK ends
