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
MODSERIAL::isr_tx(bool doCallback)
{
    if (! _base || buffer_size[TxIrq] == 0 || buffer[TxIrq] == (char *)NULL) {
        _isr[TxIrq].call(&this->callbackInfo); 
        return;
    }
    
    while (! MODSERIAL_TX_BUFFER_EMPTY && MODSERIAL_THR_HAS_SPACE ) {
        _THR = txc = (uint8_t)(buffer[TxIrq][buffer_out[TxIrq]]);
        buffer_count[TxIrq]--;   
        buffer_out[TxIrq]++;
        if (buffer_out[TxIrq] >= buffer_size[TxIrq]) {
            buffer_out[TxIrq] = 0;
        }
        if (doCallback) _isr[TxIrq].call(&this->callbackInfo);
    }
        
    if ( MODSERIAL_TX_BUFFER_EMPTY ) { 
        _IER = 1;
        _isr[TxEmpty].call(&this->callbackInfo);
    }        
}

}; // namespace AjK ends

        
