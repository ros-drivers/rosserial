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
    
    @file          example3b.cpp 
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

void rxCallback(MODSERIAL_IRQ_INFO *info) {

    // Get the pointer to our MODSERIAL object that invoked this callback.
    MODSERIAL *pc = info->serial;
    
    // info->serial points at the MODSERIAL instance so we can use it to call
    // any of the public MODSERIAL functions that are normally available. So
    // there's now no need to use the global version (pc in our case) inside
    // callback functions.    
    char c = pc->rxGetLastChar(); // Where local pc variable is a pointer to the global MODSERIAL pc object.
    
    // The following is rather daft but demos the point.
    // Don't allow the letter "A" go into the RX buffer.
    // Basically acts as a filter to remove the letter "A" 
    // if it goes into the RX buffer.
    if (c == 'A') {
        // Note, we call the MODSERIAL_IRQ_INFO::rxDiscardLastChar() public function which
        // is permitted access to the protected version of MODSERIAL::rxDiscardLastChar()
        // within MODSERIAL (because they are friends). This ensures rxDiscardLastChar()
        // can only be called within an rxCallback function. 
        info->rxDiscardLastChar(); 
    }
}

#endif
