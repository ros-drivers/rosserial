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
    // Make sure the ISR cannot use the buffers while we are manipulating them.
    NVIC_DisableIRQ(_IRQ);
    
    // If the requested size is the same as the current size there's nothing to do,
    // just continue to use the same buffer as it's fine as it is.
    if (buffer_size[type] == size)
    {
        NVIC_EnableIRQ(_IRQ);  
        return Ok;
    }
    
    // is new buffer is big enough?
    if (size <= buffer_count[type])
    {
        NVIC_EnableIRQ(_IRQ);  
        return BufferOversize;
    }
    
    // allocate new buffer
    char * newBuffer = (char*)malloc(size);
    
    // allocation failed?
    if (newBuffer == (char*)NULL)
    {
        if (memory_check)
            error("Failed to allocate memory for %s buffer", type == TxIrq ? "TX" : "RX");
            
        return NoMemory;
    }
    
    // copy old buffer content to new one
    moveRingBuffer(newBuffer, type);
    
    // free old buffer and reset ring buffer cursor
    free((char*)buffer[type]);
        
    buffer[type]      = newBuffer;
    buffer_size[type] = size;
    buffer_in[type]   = buffer_count[type];
    buffer_out[type]  = 0;    
    
    // Start the ISR system again with the new buffers.
    NVIC_EnableIRQ(_IRQ);    
    return Ok;
}

void MODSERIAL::moveRingBuffer(char * newBuffer, IrqType type)
{   
    // copy old buffer content to new one
    if(buffer_in[type] > buffer_out[type])      
    {   // content in the middle of the ring buffer
        memcpy(&newBuffer[0], (char*)&buffer[type][buffer_out[type]], buffer_count[type]);
    }  
    else if(buffer_in[type] < buffer_out[type]) 
    {   // content split, free space in the middle
        // copy last part of the old buffer
        int end_count= buffer_size[type] - buffer_out[type];
        memcpy(&newBuffer[0], (char*)&buffer[type][buffer_out[type]], end_count);
        
        // copy first part of old buffer
        memcpy(&newBuffer[end_count], (char*)buffer[type], buffer_in[type]);
    }
    // else old buffer empty
}

}; // namespace AjK ends
