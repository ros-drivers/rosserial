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

#ifndef MODSERIAL_MACROS_H
#define MODSERIAL_MACROS_H

#define MODSERIAL_RBR  0x00
#define MODSERIAL_THR  0x00
#define MODSERIAL_DLL  0x00
#define MODSERIAL_IER  0x04
#define MODSERIAL_DML  0x04
#define MODSERIAL_IIR  0x08
#define MODSERIAL_FCR  0x08
#define MODSERIAL_LCR  0x0C
#define MODSERIAL_LSR  0x14
#define MODSERIAL_SCR  0x1C
#define MODSERIAL_ACR  0x20
#define MODSERIAL_ICR  0x24
#define MODSERIAL_FDR  0x28
#define MODSERIAL_TER  0x30

#define MODSERIAL_LSR_RDR  (1UL << 0)
#define MODSERIAL_LSR_OE   (1UL << 1)
#define MODSERIAL_LSR_PE   (1UL << 2)
#define MODSERIAL_LSR_FE   (1UL << 3)
#define MODSERIAL_LSR_BR   (1UL << 4)
#define MODSERIAL_LSR_THRE (1UL << 5)
#define MODSERIAL_LSR_TEMT (1UL << 6)
#define MODSERIAL_LSR_RXFE (1UL << 7)

#define MODSERIAL_FIFO_ENABLE   1
#define MODSERIAL_FIFO_RX_RESET 2
#define MODSERIAL_FIFO_TX_RESET 4

#define _RBR    *((char *)_base+MODSERIAL_RBR)
#define _THR    *((char *)_base+MODSERIAL_THR)
#define _IIR    *((char *)_base+MODSERIAL_IIR)
#define _IER    *((char *)_base+MODSERIAL_IER)
#define _LSR    *((char *)_base+MODSERIAL_LSR)
#define _FCR    *((char *)_base+MODSERIAL_FCR)

#define MODSERIAL_TX_BUFFER_EMPTY (buffer_count[TxIrq]==0)
#define MODSERIAL_RX_BUFFER_EMPTY (buffer_count[RxIrq]==0)
#define MODSERIAL_TX_BUFFER_FULL  (buffer_count[TxIrq]==buffer_size[TxIrq])
#define MODSERIAL_RX_BUFFER_FULL  (buffer_count[RxIrq]==buffer_size[RxIrq])

#define MODSERIAL_THR_HAS_SPACE ((int)_LSR&MODSERIAL_LSR_THRE)
#define MODSERIAL_TEMT_IS_EMPTY ((int)_LSR&MODSERIAL_LSR_TEMT)
#define MODSERIAL_RBR_HAS_DATA  ((int)_LSR&MODSERIAL_LSR_RDR)

#endif
