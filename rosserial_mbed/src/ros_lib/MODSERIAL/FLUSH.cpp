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
MODSERIAL::flushBuffer(IrqType type)
{
    uint32_t irq_req = MODSERIAL_IRQ_REG;
    switch(type) {
        case TxIrq: DISABLE_TX_IRQ; break;
        case RxIrq: DISABLE_RX_IRQ; break;
        default: break;
    }
    buffer_in[type]       = 0;
    buffer_out[type]      = 0;
    buffer_count[type]    = 0;
    buffer_overflow[type] = 0;  
    switch(type) {
        case TxIrq: RESET_TX_FIFO; break;
        case RxIrq: RESET_RX_FIFO; break;
        default: break;
    }
    MODSERIAL_IRQ_REG = irq_req;
}

}; // namespace AjK ends
