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
    @version       see ChangeLog.c
    @date          Nov 2010
    @author        Andy Kirkham
*/

#ifndef MODSERIAL_H
#define MODSERIAL_H

/** @defgroup API The MODSERIAL API */
/** @defgroup MISC Misc MODSERIAL functions */
/** @defgroup INTERNALS MODSERIAL Internals */

#ifndef MODSERIAL_DEFAULT_RX_BUFFER_SIZE
#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE    256
#endif

#ifndef MODSERIAL_DEFAULT_TX_BUFFER_SIZE
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE    256
#endif

#include "mbed.h"
#include "serial_api.h"

namespace AjK {

// Forward reference.
class MODSERIAL;

/**
 * @author Andy Kirkham
 * @see http://mbed.org/cookbook/MODSERIAL
 * @see example3a.cpp
 * @see example3b.cpp
 * @see API 
 *
 * <b>MODSERIAL_IRQ_INFO</b> is a class used to pass information (and access to protected
 * MODSERIAL functions) to IRQ callbacks. 
 */
class MODSERIAL_IRQ_INFO
{
public:
    friend class MODSERIAL;
    
    MODSERIAL *serial;
    
    MODSERIAL_IRQ_INFO() { serial = 0; }
    
    /** rxDiscardLastChar()
     *
     * Remove the last char placed into the rx buffer.
     * This is an operation that can only be performed
     * by an rxCallback function. 
     * @ingroup API
     * @return The byte removed from the buffer.
     */
    int rxDiscardLastChar(void);

protected:

    /** setSerial()
     *
     * Used internally by MODSERIAL to set the "this" pointer
     * of the MODSERIAL that created this object.
     * @ingroup INTERNAL
     * @param A pointer to a MODSERIAL object instance.
     */
    void setSerial(MODSERIAL *s) { serial = s; }    
};

// Forward reference dummy class.
class MODSERIAL_callback_dummy;

/**
 * @author Andy Kirkham
 * @see http://mbed.org/cookbook/MODSERIAL
 * @see example3a.cpp
 * @see example3b.cpp
 * @see API 
 *
 * <b>MODSERIAL_callback</b> is a class used to hold application callbacks that
 * MODSERIAL can invoke on certain events.
 */
class MODSERIAL_callback 
{
protected:

    //! C callback function pointer.
    void (*c_callback)(MODSERIAL_IRQ_INFO *); 
    
    //! C++ callback object/method pointer (the object part).
    MODSERIAL_callback_dummy *obj_callback;
    
    //! C++ callback object/method pointer (the method part).
    void (MODSERIAL_callback_dummy::*method_callback)(MODSERIAL_IRQ_INFO *);

public:
    
    /** Constructor
     */
    MODSERIAL_callback() {
        c_callback      = 0;
        obj_callback    = 0;
        method_callback = 0;
    }
    
    /** attach - Overloaded attachment function.
     *
     * Attach a C type function pointer as the callback.
     *
     * Note, the callback function prototype must be:-
     * @code
     * void myCallbackFunction(MODSERIAL_IRQ_INFO *);
     * @endcode
     * @param A C function pointer to call.
     */
    void attach(void (*function)(MODSERIAL_IRQ_INFO *) = 0) { c_callback = function; }
    
    /** attach - Overloaded attachment function.
     *
     * Attach a C++ type object/method pointer as the callback.
     *
     * Note, the callback method prototype must be:-
     * @code
     *     public:
     *         void myCallbackFunction(MODSERIAL_IRQ_INFO *);
     * @endcode
     * @param A C++ object pointer.
     * @param A C++ method within the object to call.
     */
    template<class T> 
    void attach(T* item, void (T::*method)(MODSERIAL_IRQ_INFO *)) { 
        obj_callback    = (MODSERIAL_callback_dummy *)item; 
        method_callback = (void (MODSERIAL_callback_dummy::*)(MODSERIAL_IRQ_INFO *))method; 
    }

    /** call - Overloaded callback initiator.
     *
     * call the callback function.
     *
     * @param A pointer to a MODSERIAL_IRQ_INFO object.
     */
    void call(MODSERIAL_IRQ_INFO *arg) {
        if (c_callback != 0) {
            (*c_callback)(arg);
        }
        else {
            if (obj_callback  != 0 && method_callback != 0) {
                (obj_callback->*method_callback)(arg);
            }
        }       
    }    
};

/**
 * @author Andy Kirkham
 * @see http://mbed.org/cookbook/MODSERIAL
 * @see http://mbed.org/handbook/Serial
 * @see example1.cpp
 * @see example2.cpp
 * @see example3a.cpp
 * @see example3b.cpp
 * @see example_dma.cpp
 * @see API 
 *
 * <b>MODSERIAL</b> extends the Mbed library <a href="/handbook/Serial">Serial</a> to provide fully buffered
 * TX and RX streams. Buffer length is fully customisable. 
 *
 * Before using MODSERIAL users should be familar with Mbed's standard <a href="/handbook/Serial">Serial</a>
 * library object. MODSERIAL is a direct "drop in" replacement for <a href="/handbook/Serial">Serial</a>. Where
 * previously Serial was used, MODSERIAL can be used as adirect replacement instantly offering standard
 * TX and RX buffering. By default, both TX and RX buffers are 256 bytes in length.
 *
 * @image html /media/uploads/mbedofficial/serial_interfaces.png
 *
 * Standard example:
 * @code
 * #include "mbed.h"
 * #include "MODSERIAL.h"
 *
 * MODSERIAL pc(USBTX, USBRX); // tx, rx
 *
 * int main() {
 *     pc.printf("Hello World!");
 *     while(1) {
 *         pc.putc(pc.getc() + 1);
 *     }
 * }
 * @endcode
 *
 * Example with alternate buffer length:
 * @code
 * #include "mbed.h"
 * #include "MODSERIAL.h"
 *
 * // Make TX and RX buffers 512byes in length
 * MODSERIAL pc(USBTX, USBRX, 512); // tx, rx
 *
 * int main() {
 *     pc.printf("Hello World!");
 *     while(1) {
 *         pc.putc(pc.getc() + 1);
 *     }
 * }
 * @endcode
 *
 * Example with alternate buffer length:
 * @code
 * #include "mbed.h"
 * #include "MODSERIAL.h"
 *
 * // Make TX 1024bytes and RX 512byes in length
 * MODSERIAL pc(USBTX, USBRX, 1024, 512); // tx, rx
 *
 * int main() {
 *     pc.printf("Hello World!");
 *     while(1) {
 *         pc.putc(pc.getc() + 1);
 *     }
 * }
 * @endcode
 */
class MODSERIAL : public Serial 
{
public:

    // Allow instances of MODSERIAL_IRQ_INFO to use protected properties and methods.
    friend class MODSERIAL_IRQ_INFO;

    //! A copy of the Serial parity enum
    /** @see http://mbed.org/projects/libraries/api/mbed/trunk/Serial#Serial.format */
    enum Parity {
          None = 0
        , Odd
        , Even
        , Forced1   
        , Forced0
    };
    
    //! A copy of the Serial IrqType enum
    enum IrqType {
          RxIrq = 0
        , TxIrq
        , RxOvIrq
        , TxOvIrq
        , TxEmpty
        , RxAutoDetect
        , NumOfIrqTypes
    };
    
    //! Non-blocking functions return code.
    enum Result {
          Ok = 0                /*!< Ok. */
        , NoMemory       = -1   /*!< Memory allocation failed. */
        , NoChar         = -1   /*!< No character in buffer. */
        , BufferOversize = -2   /*!< Oversized buffer. */
    };
    
    /**
     * The MODSERIAL constructor is used to initialise the serial object.
     *
     * @param tx PinName of the TX pin.
     * @param rx PinName of the TX pin.
     */    
    MODSERIAL(PinName tx, PinName rx, const char* name = NULL);
    
    /**
     * The MODSERIAL constructor is used to initialise the serial object.
     *
     * @param tx PinName of the TX pin.
     * @param rx PinName of the TX pin.
     * @param bufferSize Integer of the TX and RX buffer sizes.
     */    
    MODSERIAL(PinName tx, PinName rx, int bufferSize, const char* name = NULL);
    
    /**
     * The MODSERIAL constructor is used to initialise the serial object.
     *
     * @param tx PinName of the TX pin.
     * @param rx PinName of the TX pin.
     * @param txBufferSize Integer of the TX buffer sizes.
     * @param rxBufferSize Integer of the RX buffer sizes.
     */    
    MODSERIAL(PinName tx, PinName rx, int txBufferSize, int rxBufferSize, const char* name = NULL);
    
    virtual ~MODSERIAL();

    /**
     * Function: attach
     *  
     * The Mbed standard <a href="/handbook/Serial">Serial</a> library object allows an interrupt callback
     * to be made when a byte is received by the TX or RX UART hardware. MODSERIAL traps these interrupts
     * to enable it's buffering system. However, after the byte has been received/sent under interrupt control, 
     * MODSERIAL can callback a user function as a notification of the interrupt. Note, user code should not
     * directly interact with the Uart hardware, MODSERIAL does that, instead, MODSERIAL API functions should
     * be used.
     *
     * <b>Note</b>, a character is written out then, if there is room in the TX FIFO and the TX buffer is empty,
     * putc() will put the character directly into THR (the output holding register). If the TX FIFO is full and 
     * cannot accept the character, it is placed into the TX output buffer. The TX interrupts are then enabled
     * so that when the TX FIFO empties, the TX buffer is then transferred to the THR FIFO. The TxIrq will ONLY 
     * be activated when this transfer of a character from BUFFER to THR FIFO takes place. If your character 
     * throughput is not high bandwidth, then the 16 byte TX FIFO may be enough and the TX output buffer may 
     * never come into play.
     *
     * @code
     * #include "mbed.h"
     * #include "MODSERIAL.h"
     *
     * DigitalOut led1(LED1);
     * DigitalOut led2(LED2);
     * DigitalOut led3(LED3);
     *
     * // To test, connect p9 to p10 as a loopback.
     * MODSERIAL pc(p9, p10);
     *
     * // This function is called when a character goes into the TX buffer.
     * void txCallback(void) {
     *     led2 = !led2;
     * }
     *
     * // This function is called when a character goes into the RX buffer.
     * void rxCallback(void) {
     *     led3 = !led3;
     * }
     *
     * int main() {
     *     pc.baud(115200);
     *     pc.attach(&txCallback, MODSERIAL::TxIrq);
     *     pc.attach(&rxCallback, MODSERIAL::RxIrq);
     *
     *     while(1) {
     *         led1 = !led1;
     *         wait(0.5);
     *         pc.putc('A');
     *         wait(0.5);
     *     }
     * ]
     * @endcode
     *
     * @ingroup API
     * @param fptr A pointer to a void function, or 0 to set as none
     * @param type Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */  
    void attach(void (*fptr)(MODSERIAL_IRQ_INFO *), IrqType type = RxIrq) { _isr[type].attach(fptr); }
    
    /**
     * Function: attach
     *  
     * The Mbed standard <a href="/handbook/Serial">Serial</a> library object allows an interrupt callback
     * to be made when a byte is received by the TX or RX UART hardware. MODSERIAL traps these interrupts
     * to enable it's buffering system. However, after the byte has been received/sent under interrupt control, 
     * MODSERIAL can callback a user function as a notification of the interrupt. Note, user code should not
     * directly interact with the Uart hardware, MODSERIAL does that, instead, MODSERIAL API functions should
     * be used.
     *
     * <b>Note</b>, a character is written out then, if there is room in the TX FIFO and the TX buffer is empty,
     * putc() will put the character directly into THR (the output holding register). If the TX FIFO is full and 
     * cannot accept the character, it is placed into the TX output buffer. The TX interrupts are then enabled
     * so that when the TX FIFO empties, the TX buffer is then transferred to the THR FIFO. The TxIrq will ONLY 
     * be activated when this transfer of a character from BUFFER to THR FIFO takes place. If your character 
     * throughput is not high bandwidth, then the 16 byte TX FIFO may be enough and the TX output buffer may 
     * never come into play.
     *
     * @code
     * #include "mbed.h"
     * #include "MODSERIAL.h"
     *
     * DigitalOut led1(LED1);
     * DigitalOut led2(LED2);
     * DigitalOut led3(LED3);
     *
     * // To test, connect p9 to p10 as a loopback.
     * MODSERIAL pc(p9, p10);
     *
     * class Foo {
     * public:
     *     // This method is called when a character goes into the TX buffer.
     *     void txCallback(void) { led2 = !led2; }
     *
     *     // This method is called when a character goes into the RX buffer.
     *     void rxCallback(void) { led3 = !led3; }
     * };
     *
     * Foo foo;
     *
     * int main() {
     *     pc.baud(115200);
     *     pc.attach(&foo, &Foo::txCallback, MODSERIAL::TxIrq);
     *     pc.attach(&foo, &Foo::rxCallback, MODSERIAL::RxIrq);
     *
     *     while(1) {
     *         led1 = !led1;
     *         wait(0.5);
     *         pc.putc('A');
     *         wait(0.5);
     *     }
     * ]
     * @endcode
     *     
     * @ingroup API
     * @param  tptr A pointer to the object to call the member function on
     * @param  mptr A pointer to the member function to be called
     * @param  type Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(MODSERIAL_IRQ_INFO *), IrqType type = RxIrq) {
        if((mptr != 0) && (tptr != 0)) {
            _isr[type].attach(tptr, mptr);            
        }
    }

    /**
     * @see attach
     * @ingroup API
     */
    void connect(void (*fptr)(MODSERIAL_IRQ_INFO *), IrqType type = RxIrq) { _isr[RxIrq].attach(fptr); }
    
    /**
     * @see attach
     * @ingroup API
     */
    template<typename T>
    void connect(T* tptr, void (T::*mptr)(MODSERIAL_IRQ_INFO *), IrqType type = RxIrq) {
        if((mptr != 0) && (tptr != 0)) {
            _isr[type].attach(tptr, mptr);            
        }
    }
    
    /**
     * Function: writeable
     *  
     * Determine if there is space available to write a byte
     *
     * @ingroup API
     * @return 1 if there is space to write a character, else 0
     */
    int writeable() { return txBufferFull() ? 0 : 1; }
    
    /**
     * Function: readable
     *  
     * Determine if there is a byte available to read
     *
     * @ingroup API
     * @return 1 if there is a character available to read, else 0
     */
    int readable() { return rxBufferEmpty() ? 0 : 1; } 
    
    /**
     * Function: txBufferSane
     *  
     * Determine if the TX buffer has been initialized.
     *
     * @ingroup API
     * @return true if the buffer is initialized, else false
     */
    bool txBufferSane(void) { return buffer[TxIrq] != (char *)NULL ? true : false; }
    
    /**
     * Function: rxBufferSane
     *  
     * Determine if the RX buffer has been initialized.
     *
     * @ingroup API
     * @return true if the buffer is initialized, else false
     */
    bool rxBufferSane(void) { return buffer[TxIrq] != (char *)NULL ? true : false; }
    
    /**
     * Function: txBufferGetCount
     *  
     * Returns how many bytes are in the TX buffer
     *
     * @ingroup API
     * @return The number of bytes in the TX buffer
     */
    int txBufferGetCount(void)    { return buffer_count[TxIrq]; }
    
    /**
     * Function: rxBufferGetCount
     *  
     * Returns how many bytes are in the RX buffer
     *
     * @ingroup API
     * @return The number of bytes in the RX buffer
     */
    int rxBufferGetCount(void)    { return buffer_count[RxIrq]; }
    
    /**
     * Function: txBufferGetSize
     *  
     * Returns the current size of the TX buffer
     *
     * @ingroup API
     * @return The length iof the TX buffer in bytes
     */
    int txBufferGetSize(int size) { return buffer_size[TxIrq]; } 
    
    /**
     * Function: rxBufferGetSize
     *  
     * Returns the current size of the RX buffer
     *
     * @ingroup API
     * @return The length iof the RX buffer in bytes
     */
    int rxBufferGetSize(int size) { return buffer_size[RxIrq]; } 
    
    /**
     * Function: txBufferFull
     *  
     * Is the TX buffer full?
     *
     * @ingroup API
     * @return true if the TX buffer is full, otherwise false
     */
    bool txBufferFull(void);
    
    /**
     * Function: rxBufferFull
     *  
     * Is the RX buffer full?
     *
     * @ingroup API
     * @return true if the RX buffer is full, otherwise false
     */
    bool rxBufferFull(void);
    
    /**
     * Function: txBufferEmpty
     *  
     * Is the TX buffer empty?
     *
     * @ingroup API
     * @return true if the TX buffer is empty, otherwise false
     */
    bool txBufferEmpty(void);
    
    /**
     * Function: rxBufferEmpty
     *  
     * Is the RX buffer empty?
     *
     * @ingroup API
     * @return true if the RX buffer is empty, otherwise false
     */
    bool rxBufferEmpty(void);
    
    /**
     * Function: txBufferSetSize
     *  
     * Change the TX buffer size.
     *
     * @see Result
     * @ingroup API
     * @param size The new TX buffer size in bytes.
     * @param m Perform a memory sanity check. Errs the Mbed if memory alloc fails.
     * @return Result Ok on success.
     */
    int txBufferSetSize(int size, bool m) { return resizeBuffer(size, TxIrq, m); } 
    
    /**
     * Function: rxBufferSetSize
     *  
     * Change the RX buffer size.
     *
     * @see Result
     * @ingroup API
     * @param size The new RX buffer size in bytes.
     * @param m Perform a memory sanity check. Errs the Mbed if memory alloc fails.
     * @return Result Ok on success.
     */
    int rxBufferSetSize(int size, bool m) { return resizeBuffer(size, RxIrq, m); } 
    
    /**
     * Function: txBufferSetSize
     *  
     * Change the TX buffer size.
     * Always performs a memory sanity check, halting the Mbed on failure.
     *
     * @see Result
     * @ingroup API
     * @param size The new TX buffer size in bytes.
     * @return Result Ok on success.
     */
    int txBufferSetSize(int size) { return resizeBuffer(size, TxIrq, true); } 
    
    /**
     * Function: rxBufferSetSize
     *  
     * Change the RX buffer size.
     * Always performs a memory sanity check, halting the Mbed on failure.
     *
     * @see Result
     * @ingroup API
     * @param size The new RX buffer size in bytes.
     * @return Result Ok on success.
     */
    int rxBufferSetSize(int size) { return resizeBuffer(size, RxIrq, true); } 
    
    /**
     * Function: txBufferFlush
     *  
     * Remove all bytes from the TX buffer.
     * @ingroup API
     */
    void txBufferFlush(void) { flushBuffer(TxIrq); }
    
    /**
     * Function: rxBufferFlush
     *  
     * Remove all bytes from the RX buffer.
     * @ingroup API
     */
    void rxBufferFlush(void) { flushBuffer(RxIrq); }
        
    /**
     * Function: getcNb
     *
     * Like getc() but is non-blocking. If no bytes are in the RX buffer this
     * function returns Result::NoChar (-1)
     *
     * @ingroup API
     * @return A byte from the RX buffer or Result::NoChar (-1) if bufer empty.
     */
    int getcNb() { return __getc(false); }
    
    /**
     * Function: getc
     *
     * Overloaded version of Serial::getc()
     * 
     * This function blocks (if the RX buffer is empty the function will wait for a
     * character to arrive and then return that character).
     *
     * @ingroup API
     * @return A byte from the RX buffer
     */
    int getc()   { return __getc(true);  }
    
    /**
     * Function: txGetLastChar
     *
     * Rteurn the last byte to pass through the TX interrupt handler.
     *
     * @ingroup MISC
     * @return The byte
     */
    char txGetLastChar(void) { return txc; }
    
    /**
     * Function: rxGetLastChar
     *
     * Return the last byte to pass through the RX interrupt handler.
     *
     * @ingroup MISC
     * @return The byte
     */
    char rxGetLastChar(void) { return rxc; }
    

    
    /**
     * Function: autoDetectChar
     *
     * Set the char that, if seen incoming, invokes the AutoDetectChar callback.
     *
     * @ingroup API
     * @param int c The character to detect.
     */
    void autoDetectChar(char c) { auto_detect_char = c; }
    
    /**
     * Function: move
     *
     * Move contents of RX buffer to external buffer. Stops if "end" detected.
     *
     * @ingroup API
     * @param char *s The destination buffer address
     * @param int max The maximum number of chars to move.
     * @param char end If this char is detected stop moving.
     */
    int move(char *s, int max, char end) {
        int counter = 0;
        char c;
        while(readable()) {
            c = getc();
            if (c == end) break;
            *(s++) = c;
            counter++;
            if (counter == max) break;
        }
        return counter;
    }
    
    /**
     * Function: move (overloaded)
     *
     * Move contents of RX buffer to external buffer. Stops if auto_detect_char detected.
     *
     * @ingroup API
     * @param int max The maximum number of chars to move.
     * @param char *s The destination buffer address
     */
    int move(char *s, int max) {
        return move(s, max, auto_detect_char);
    }
    
    /**
    * Function: claim
    *
    * Redirect a stream to this MODSERIAL object
    *
    * Important: A name parameter must have been added when creating the MODSERIAL object:
    *
    * @code
    * #include "MODSERIAL.h"
    * ...
    * MODSERIAL pc(USBTX, USBRX, "modser");
    * 
    * int main() {
    *   pc.claim()            // capture <stdout>
    *   pc.printf("Uses the MODSERIAL library\r\n");
    *   printf("So does this!\r\n");
    * }
    * @endcode
    *
    * @ingroup API
    * @param FILE *stream The stream to redirect (default = stdout)
    * @return true if succeeded, else false
    */    
    bool claim(FILE *stream = stdout);
    
    #if 0 // Inhereted from Serial/Stream, for documentation only
    /**
     * Function: putc
     * 
     * Write a character
     * Inhereted from Serial/Stream
     *
     * @see http://mbed.org/projects/libraries/api/mbed/trunk/Serial#Serial.putc
     * @ingroup API
     * @param c The character to write to the serial port
     */
    int putc(int c);
    #endif
    
    #if 0 // Inhereted from Serial/Stream, for documentation only
    /**
     * Function: printf
     *  
     * Write a formated string
     * Inhereted from Serial/Stream
     *
     * @see http://mbed.org/projects/libraries/api/mbed/trunk/Serial#Serial.printf
     * @ingroup API
     * @param format A printf-style format string, followed by the variables to use in formating the string.
     */
    int printf(const char* format, ...);
    #endif
    
    #if 0 // Inhereted from Serial/Stream, for documentation only
    /**
     * Function: scanf
     *  
     * Read a formated string
     * Inhereted from Serial/Stream
     *
     * @see http://mbed.org/projects/libraries/api/mbed/trunk/Serial#Serial.scanf
     * @ingroup API
     * @param format - A scanf-style format string, followed by the pointers to variables to store the results.
     */
    int scanf(const char* format, ...);
    #endif

protected:    
    /**
     * Used to pass information to callbacks.
     * @ingroup INTERNALS
     */
    MODSERIAL_IRQ_INFO callbackInfo;

    /**
     * Remove the last char placed into the rx buffer.
     * This is an operation that can only be performed
     * by an rxCallback function. To protect the buffers
     * this function is defined protected so that a 
     * regular application cannot call it directly. It 
     * can only be called by the public version within a
     * MODSERIAL_IRQ_INFO class.
     * @ingroup INTERNALS
     * @return The byte removed from the buffer.
     */
    int rxDiscardLastChar(void);    
            
private:

    /**
     * A pointer to the UART peripheral base address being used.
     * @ingroup INTERNALS
     */
    void *_base;
    
    /**
     * The last byte to pass through the TX IRQ handler.
     * @ingroup INTERNALS
     */
    volatile char txc;
    
    /**
     * The last byte to pass through the RX IRQ handler.
     * @ingroup INTERNALS
     */
    volatile char rxc;
    
    /**
     * Pointers to the TX and RX buffers.
     * @ingroup INTERNALS
     */
    volatile char *buffer[2];
    
    /**
     * Buffer in pointers.
     * @ingroup INTERNALS
     */
    volatile int   buffer_in[2];
    
    /**
     * Buffer out pointers.
     * @ingroup INTERNALS
     */
    volatile int   buffer_out[2];
    
    /**
     * Buffer lengths.
     * @ingroup INTERNALS
     */
    volatile int   buffer_size[2];
    
    /**
     * Buffer content counters.
     * @ingroup INTERNALS
     */
    volatile int   buffer_count[2];
    
    /**
     * Buffer overflow.
     * @ingroup INTERNALS
     */
    volatile int   buffer_overflow[2];
    
    /**
     * Auto-detect character.
     * @ingroup INTERNALS
     */
    volatile char auto_detect_char;
    
    /**
     * Callback system.
     * @ingroup INTERNALS
     */
    MODSERIAL_callback _isr[NumOfIrqTypes];
    
    /**
     * TX Interrupt Service Routine.
     * @ingroup INTERNALS
     */
    void isr_tx(bool doCallback);
    
    /**
     * TX Interrupt Service Routine stub version.
     * @ingroup INTERNALS
     */ 
    void isr_tx(void) { isr_tx(true); }
    
    
    /**
     * RX Interrupt Service Routine.
     * @ingroup INTERNALS
     */
    void isr_rx(void);
    
        /**
     * Get a character from the RX buffer
     * @ingroup INTERNALS
     * @param bool True to block (wait for input)
     * @return A byte from the buffer.
     */
    int __getc(bool);
    
    /**
     * Put a character from the TX buffer
     * @ingroup INTERNALS
     * @param bool True to block (wait for space in the TX buffer if full)
     * @return 0 on success
     */
    int __putc(int c, bool);
    
    /**
     * Function: _putc 
     * Overloaded virtual function.
     */
    virtual int _putc(int c) { return __putc(c, true); }
    
    /**
     * Function: _getc 
     * Overloaded virtual function.
     */
    virtual int _getc()      { return __getc(true); }
        
    /** 
     * Function: init
     * Initialize the MODSERIAL object
     * @ingroup INTERNALS
     */
    void init(int txSize, int rxSize, PinName rx);
    
    /** 
     * Function: flushBuffer
     * @ingroup INTERNALS
     */
    void flushBuffer(IrqType type);

    /** 
     * Function: resizeBuffer
     * @ingroup INTERNALS
     */
    int resizeBuffer(int size, IrqType type = RxIrq, bool memory_check = true);   
       
    /** 
     * Function: moveRingBuffer
     * @ingroup INTERNALS
     */
    void moveRingBuffer(char * newBuffer, IrqType type);
    
    


//DEVICE SPECIFIC FUNCTIONS:
    private:
    /**
    * Set pointers to UART and IRQ
    */
    void setBase( void );
    
    /**
    * If required, allows for adding specific settings
    */
    void initDevice( void );
    
    IRQn_Type _IRQ;
    
    public:
     /**
     * Function: txIsBusy
     *
     * If the Uart is still actively sending characters this
     * function will return true.
     *
     * @ingroup API
     * @return bool
     */
    bool txIsBusy(void);
    

};

}; // namespace AjK ends

using namespace AjK;

#endif

