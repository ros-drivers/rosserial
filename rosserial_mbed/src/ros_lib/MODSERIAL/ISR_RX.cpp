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
MODSERIAL::isr_rx(void)
{
    if (! _base || buffer_size[RxIrq] == 0 || buffer[RxIrq] == (char *)NULL) {
        _isr[RxIrq].call(&this->callbackInfo); 
        return;
    } 
    
    while( MODSERIAL_RBR_HAS_DATA ) {
        rxc = (char)(_RBR & 0xFF); 
        if ( MODSERIAL_RX_BUFFER_FULL ) {
            buffer_overflow[RxIrq] = rxc; // Oh dear, no room in buffer.
            _isr[RxOvIrq].call(&this->callbackInfo);
        }
        else {
            if (buffer[RxIrq] != (char *)NULL) {
                buffer[RxIrq][buffer_in[RxIrq]] = rxc;
                buffer_count[RxIrq]++; 
                buffer_in[RxIrq]++;
                if (buffer_in[RxIrq] >= buffer_size[RxIrq]) {
                    buffer_in[RxIrq] = 0;
                }
            }  
            _isr[RxIrq].call(&this->callbackInfo); 
        }
        if (auto_detect_char == rxc) {
            _isr[RxAutoDetect].call(&this->callbackInfo);
        }
    }    
}

}; // namespace AjK ends

