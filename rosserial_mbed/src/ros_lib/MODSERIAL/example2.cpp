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
    
    @file          example2.cpp 
    @purpose       Demos a simple messaging system.
    @version       see ChangeLog.c
    @date          Jan 2011
    @author        Andy Kirkham
*/

/*
    This example demostrates a simple "messaging" system. You can use it with
    a terminal program to test it out or write a cusom C#/C++/VB/etc program
    to read and write messages to or from the Mbed. The default baud rate in
    this example is 115200.
    
    In this example, the LEDs are controlled and pins p21 to p24 are set as
    InterruptIn and send messages out when their value changes.
    
    To use, hook up the MBed USB and open your fav terminal. All messages
    end with the \n character, don't forget to hit carriage return!. 
    As an example:-
        
        to switch on LED1 send LED1:1\n, off is LED1:0\n and toggle is LED1:2\n
        to switch on LED2 send LED2:1\n, off is LED2:0\n and toggle is LED2:2\n
        to switch on LED3 send LED3:1\n, off is LED3:0\n and toggle is LED3:2\n
        to switch on LED4 send LED4:1\n, off is LED4:0\n and toggle is LED4:2\n
        
    When a pin change on p21 to p24 happens, a message is sent. As an example
    when p21 goes low PIN21:0\n is sent, when goes high PIN21:1\n is sent.
    
    Note, the InterruptIn pins p21 to p24 are setup to have pullups. This means
    they are high. To activate them use a wire to short the pin to 0volts.
    
    If you find that p21 to p24 sent a lot of on/off/on/off then it's probably
    due to "bounce". If you are connecting a mechanical switch to a pin you
    may prefer to use the PinDetect library rather than using InterruptIn.
    @see http://mbed.org/users/AjK/libraries/PinDetect/latest
    
    One point you may notice. Incoming messages are processed via main()'s
    while(1) loop whereas pin changes have their messages directly sent.
    The reason for this is when MODSERIAL makes callbacks to your application
    it is in "interrupt context". And one thing you want to avoid is spending
    lots of CPU time in that context. So, the callback moves the message from
    the input buffer to a local holding buffer and it then sets a bool flag
    which tells main()'s while(1) loop to process that buffer. This means the 
    time spent doing the real incoming message handing is within your program
    and not within MODSERIAL's interrupt context. So you may ask, why not do
    the same for out going messages? Well, because MODSERIAL output buffers
    all your sent content then sending chars is very fast. MODSERIAL handles
    all the nitty gritty bits for you. You can just send. This example uses
    puts() to send the message. If you can, always try and use sprintf()+puts()
    rathe than printf(), printf() is known to often screw things up when used
    within an interrupt context. Better still, just use puts() and do away
    with any of the crappy ?printf() calls if possible. But I found the code
    below to work fine even at 115200baud.
    
*/


#ifdef COMPILE_EXAMPLE1_CODE_MODSERIAL

#include "mbed.h"
#include "MODSERIAL.h"

#define MESSAGE_BUFFER_SIZE 32

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

InterruptIn P21(p21);
InterruptIn P22(p22);
InterruptIn P23(p23);
InterruptIn P24(p24);

MODSERIAL messageSystem(USBTX, USBRX);

char messageBufferIncoming[MESSAGE_BUFFER_SIZE];
char messageBufferOutgoing[MESSAGE_BUFFER_SIZE];
bool messageReceived;

void messageReceive(MODSERIAL_IRQ_INFO *q) {
    MODSERIAL *sys = q->serial;
    sys->move(messageBufferIncoming, MESSAGE_BUFFER_SIZE);
    messageReceived = true;
    return 0;
}

void messageProcess(void) {
         if (!strncmp(messageBufferIncoming, "LED1:1", sizeof("LED1:1")-1)) led1 = 1;
    else if (!strncmp(messageBufferIncoming, "LED1:0", sizeof("LED1:0")-1)) led1 = 0;
    else if (!strncmp(messageBufferIncoming, "LED1:2", sizeof("LED1:2")-1)) led1 = !led1;
    
    else if (!strncmp(messageBufferIncoming, "LED2:1", sizeof("LED2:1")-1)) led2 = 1;
    else if (!strncmp(messageBufferIncoming, "LED2:0", sizeof("LED2:0")-1)) led2 = 0;
    else if (!strncmp(messageBufferIncoming, "LED2:2", sizeof("LED2:2")-1)) led2 = !led2;
    
    else if (!strncmp(messageBufferIncoming, "LED3:1", sizeof("LED3:1")-1)) led3 = 1;
    else if (!strncmp(messageBufferIncoming, "LED3:0", sizeof("LED3:0")-1)) led3 = 0;
    else if (!strncmp(messageBufferIncoming, "LED3:2", sizeof("LED3:2")-1)) led3 = !led3;
    
    else if (!strncmp(messageBufferIncoming, "LED4:1", sizeof("LED4:1")-1)) led4 = 1;
    else if (!strncmp(messageBufferIncoming, "LED4:0", sizeof("LED4:0")-1)) led4 = 0;
    else if (!strncmp(messageBufferIncoming, "LED4:2", sizeof("LED4:2")-1)) led4 = !led4;
    
    messageReceived = false;
}

#define PIN_MESSAGE_SEND(x,y) \
    sprintf(messageBufferOutgoing,"PIN%02d:%d\n",x,y);\
    messageSystem.puts(messageBufferOutgoing);

void pin21Rise(void) { PIN_MESSAGE_SEND(21, 1); }
void pin21Fall(void) { PIN_MESSAGE_SEND(21, 0); }
void pin22Rise(void) { PIN_MESSAGE_SEND(22, 1); }
void pin22Fall(void) { PIN_MESSAGE_SEND(22, 0); }
void pin23Rise(void) { PIN_MESSAGE_SEND(23, 1); }
void pin23Fall(void) { PIN_MESSAGE_SEND(23, 0); }
void pin24Rise(void) { PIN_MESSAGE_SEND(24, 1); }
void pin24Fall(void) { PIN_MESSAGE_SEND(24, 0); }

int main() {

    messageReceived = false;
    messageSystem.baud(115200);
    messageSystem.attach(&messageReceive, MODSERIAL::RxAutoDetect);
    messageSystem.autoDetectChar('\n'); 

    // Enable pullup resistors on pins.
    P21.mode(PullUp); P22.mode(PullUp); P23.mode(PullUp); P24.mode(PullUp);
    
    // Fix Mbed library bug, see http://mbed.org/forum/bugs-suggestions/topic/1498
    LPC_GPIOINT->IO2IntClr = (1UL << 5) | (1UL << 4) | (1UL << 3) | (1UL << 2); 
    
    // Attach InterruptIn pin callbacks.
    P21.rise(&pin21Rise); P21.fall(&pin21Fall);
    P22.rise(&pin22Rise); P22.fall(&pin22Fall);
    P23.rise(&pin23Rise); P23.fall(&pin23Fall);
    P24.rise(&pin24Rise); P24.fall(&pin24Fall);
    
    while(1) {
        // Process incoming messages.
        if (messageReceived) messageProcess();
    }
}

#endif
