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
    
    @file          MODSERIAL.h 
    @purpose       Extends Serial to provide fully buffered IO
    @version       1.6
    @date          Nov 2010
    @author        Andy Kirkham    
*/

#include "MODSERIAL.h"
#include "MACROS.h"

namespace AjK {

MODSERIAL::MODSERIAL( PinName tx, PinName rx, const char* name ) : Serial( tx, rx, name )
{
    init( MODSERIAL_DEFAULT_TX_BUFFER_SIZE, MODSERIAL_DEFAULT_RX_BUFFER_SIZE, rx );
}

MODSERIAL::MODSERIAL( PinName tx, PinName rx, int bufferSize, const char* name ) : Serial( tx, rx, name )
{
    init( bufferSize, bufferSize, rx );
}

MODSERIAL::MODSERIAL( PinName tx, PinName rx, int txSize, int rxSize, const char* name ) : Serial( tx, rx, name )
{
    init( txSize, rxSize, rx );
}

MODSERIAL::~MODSERIAL()
{
    NVIC_DisableIRQ(_IRQ);
    if ( buffer[0] != NULL) free((char *)buffer[0] );
    if ( buffer[1] != NULL) free((char *)buffer[1] );    
}

bool MODSERIAL::txBufferFull( void ) 
{ 
    return MODSERIAL_TX_BUFFER_FULL; 
}

bool MODSERIAL::rxBufferFull( void ) 
{ 
    return MODSERIAL_RX_BUFFER_FULL; 
}

bool MODSERIAL::txBufferEmpty( void ) 
{ 
    return MODSERIAL_TX_BUFFER_EMPTY; 
}

bool MODSERIAL::rxBufferEmpty( void ) 
{ 
    return MODSERIAL_RX_BUFFER_EMPTY; 
}


int MODSERIAL::rxDiscardLastChar( void )
{
    // This function can only be called indirectly from
    // an rxCallback function. Therefore, we know we 
    // just placed a char into the buffer.
    char c = buffer[RxIrq][buffer_in[RxIrq]];
    
    if (buffer_count[RxIrq]) {        
        buffer_count[RxIrq]--;
        buffer_in[RxIrq]--;
        if (buffer_in[RxIrq] < 0) {
            buffer_in[RxIrq] = buffer_size[RxIrq] - 1;
        }
    }
    
    return (int)c;
}


bool MODSERIAL::claim (FILE *stream) {
    if ( _name == NULL) {
        error("claim requires a name to be given in the instantiator of the MODSERIAL instance!\r\n");
    }
    
    //Add '/' before name:
    char *path = new char[strlen(_name) + 2];
    sprintf(path, "/%s", _name);
    
    if (freopen(path, "w", stream) == NULL) {
        // Failed, should not happen
        return false;
    }
    
    delete(path);
    
    //No buffering
    setvbuf(stdout, NULL, _IONBF, buffer_size[TxIrq]);
    return true;
} 



}; // namespace AjK ends
