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

#include "MODSERIAL_LPC1768.h"
#include "MODSERIAL_LPC11U24.h"
#include "MODSERIAL_KL25Z.h"
#include "MODSERIAL_KL05Z.h"
#include "MODSERIAL_KSDK.h"
#include "MODSERIAL_NUCLEO_F401RE.h"
#include "MODSERIAL_PAC_F401RB.h"

#define MODSERIAL_TX_BUFFER_EMPTY (buffer_count[TxIrq]==0)
#define MODSERIAL_RX_BUFFER_EMPTY (buffer_count[RxIrq]==0)
#define MODSERIAL_TX_BUFFER_FULL  (buffer_count[TxIrq]==buffer_size[TxIrq])
#define MODSERIAL_RX_BUFFER_FULL  (buffer_count[RxIrq]==buffer_size[RxIrq])

#endif
