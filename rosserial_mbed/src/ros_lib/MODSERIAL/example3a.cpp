/*
    Copyright (c) 2011 Andy Kirkham
 
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
    
    @file          example3.cpp 
    @purpose       Demos a simple filter.
    @version       see ChangeLog.c
    @author        Andy Kirkham
*/

/*
    This example shows how to use the new callback system. In the old system
    Mbed's FunctionPointer[1] type was used to store abd make calls to callbacks.
    However, that limits the callback function prototype to void func(void);
    which means we cannot pass parameters.
    
    This latest version of MODSERIAL now uses its own callback object. This allows
    the passing of a pointer to a class that holds information about the MODSERIAL
    object making the callback. As of version 1.18 one critcal piece of information
    is passed, a pointer to the MODSERIAL object. This allows callbacks to use the
    MODSERIAL functions and data.
        
    Additionally, since MODSERIAL and the callback parameter class MODSERIAL_IRQ_INFO
    are friends, MODSERIAL_IRQ_INFO can access the protected functions of MODSERIAL.
    This is used to ensure functions that can only be called during a callback
    can be invoked from a callback. 
    
    [1] http://mbed.org/projects/libraries/svn/mbed/trunk/FunctionPointer.h    
*/

#ifdef COMPILE_EXAMPLE3_CODE_MODSERIAL

#include "mbed.h"
#include "MODSERIAL.h"

DigitalOut led1(LED1);

MODSERIAL pc(USBTX, USBRX);

// The following callback is defined in example3b.cpp
//! @see example3b.cpp
void rxCallback(MODSERIAL_IRQ_INFO *info);

int main() {
    
    int life_counter = 0;
    
    pc.baud(115200);
    
    pc.attach(&rxCallback, MODSERIAL::RxIrq);

    while(1) {
        // Echo back any chars we get except 'A' which is filtered by the rxCallback.
        if (pc.readable()) {
            pc.putc(pc.getc());
        }
        
        // Toggle LED1 every so often to show we are alive.
        if (life_counter++ == 1000000) {
            life_counter = 0;
            led1 = !led1;
        }
    }
}

#endif
