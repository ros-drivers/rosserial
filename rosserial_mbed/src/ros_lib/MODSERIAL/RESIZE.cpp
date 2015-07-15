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
MODSERIAL::resizeBuffer(int size, IrqType type, bool memory_check)
{
    int rval = Ok;
    
    // If the requested size is the same as the current size there's nothing to do,
    // just continue to use the same buffer as it's fine as it is.
    if (buffer_size[type] == size) return rval;
    
    // Make sure the ISR cannot use the buffers while we are manipulating them.
    disableIrq();
    
    // If the requested buffer size is larger than the current size, 
    // attempt to create a new buffer and use it.
    if (buffer_size[type] < size) {
        rval = upSizeBuffer(size, type, memory_check);
    }
    else if (buffer_size[type] > size) {
        rval = downSizeBuffer(size, type, memory_check);
    }
    
    // Start the ISR system again with the new buffers.
    enableIrq();
    
    return rval;
}

int 
MODSERIAL::downSizeBuffer(int size, IrqType type, bool memory_check)
{
    if (size >= buffer_count[type]) {
        return BufferOversize;
    }
    
    char *s = (char *)malloc(size);
    
    if (s == (char *)NULL) {
        if (memory_check) {
            error("Failed to allocate memory for %s buffer", type == TxIrq ? "TX" : "RX");
        }
        return NoMemory;
    }
    
    int c, new_in = 0;
    
    do {
        c = __getc(false);
        if (c != -1) s[new_in++] = (char)c;
        if (new_in >= size) new_in = 0;
    }
    while (c != -1);
    
    free((char *)buffer[type]);
    buffer[type]      = s;
    buffer_in[type]   = new_in;
    buffer_out[type]  = 0;
    return Ok;        
}

int 
MODSERIAL::upSizeBuffer(int size, IrqType type, bool memory_check)
{
    char *s = (char *)malloc(size);
    
    if (s == (char *)NULL) {
        if (memory_check) {
            error("Failed to allocate memory for %s buffer", type == TxIrq ? "TX" : "RX");
        }
        return NoMemory;
    }
    
    if (buffer_count[type] == 0) { // Current buffer empty?
        free((char *)buffer[type]);
        buffer[type]      = s;
        buffer_in[type]   = 0;
        buffer_out[type]  = 0;
    }        
    else { // Copy the current contents into the new buffer.
        int c, new_in = 0;
        do {
            c = __getc(false);
            if (c != -1) s[new_in++] = (char)c;
            if (new_in >= size) new_in = 0; // Shouldn't happen, but be sure.
        }
        while (c != -1);
        free((char *)buffer[type]);
        buffer[type]      = s;
        buffer_in[type]   = new_in;
        buffer_out[type]  = 0;
    }
    
    buffer_size[type] = size;
    return Ok;
}

}; // namespace AjK ends
