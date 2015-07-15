/* $Id:$

1.25    8th January 2013
    
    * Bring back into line with MBed libraries.
    * Credits: 
        Erik Olieman : http://mbed.org/users/Sissors/code/MODSERIAL/rev/3ba4341d74d6
        Erik Olieman : http://mbed.org/users/Sissors/code/MODSERIAL/rev/a469aa702bab
    

1.24    6th Dec 2012
        
    * Beta release for new Mbed library.

1.23    25th July 2012

    * LPC1768 code as was. This release includes "alpha" support for the LPC11U24

1.22    19th April 2012

    * http://mbed.org/forum/bugs-suggestions/topic/2936/
    * Bug fix, protect important buffer pointers from IRQ corruption.
    * Credits: 
        Anthony Wieser  http://mbed.org/users/WieserSoftwareLtd/ for the fix.
        BlazeX http://mbed.org/users/BlazeX/ for the alert that a fix was needed!

1.21    10 May 2011
    
    * http://mbed.org/forum/mbed/topic/2264
    
1.20    26 April 2011

    * Bug fix, not blocking on transmit
      by Erik Petrich, http://mbed.org/forum/bugs-suggestions/topic/2200
      
1.19    20 April 2011

    * Fixed some doxygen comment bugs.
    
1.18    20 April 2011

    * All callbacks now use MODSERIAL_callback (rather than Mbed's FunctionPointer[1] type)
      to store and invoke it's callbacks. This allows MODSERIAL to pass a parameter
      to callbacks. The function prototype is now void func(MODSERIAL_IRQ_INFO *q).
    * Callbacks now pass a pointer to a MODSERIAL_IRQ_INFO class type.
      This class holds a pointer to the MODSERIAL object that invoked the callback
      thus freeing callbacks need to use the global variable of the original 
      MODSERIAL instance.
    * MODSERIAL_IRQ_INFO also declares public functions that are protected within MODSERIAL
      thus allowing certain functions to be restricted to callback context only.
    * New function MODSERIAL_IRQ_INFO::rxDiscardLastChar() allows an rxCallback function
      to remove the character that was just placed into the RX buffer.
    
    [1] http://mbed.org/users/AjK/libraries/FPointer/latest/docs/

1.17   08/Mar/2011
       Fixed a memory leak in the DMA code.
       
1.16 - 12 Feb 2011
    
    * Missed one, doh!

1.15 - 12 Feb 2011
    
    * Fixed some typos.
    
1.14 - 7 Feb 2011

    * Fixed a bug in __putc() that caused the output buffer pointer to 
      become corrupted.

1.13 - 20/01/2011

    * Added extra documentation.
    * Fixed some typos.
    
1.12 - 20/01/2011

    * Added new "autoDetectChar()" function. To use:-
      1st: Add a callback to invoke when the char is detected:-        
        .attach(&detectedChar, MODSERIAL::RxAutoDetect);
      2nd: Send the char to detect.
        .autoDectectChar('\n');
      Whenever that char goes into the RX buffer your callback will be invoked.
      Added example2.cpp to demo a simple messaging system using this auto feature.


1.11 - 23/11/2010

    * Fixed a minor issue with 1.10 missed an alteration of name change.
    
1.10 - 23/11/2010

    * Rename the DMA callback from attach_dma_complete() to attach_dmaSendComplete()
    
1.9 - 23/11/2010

    * Added support for DMA sending of characters. Required is
      the MODDMA library module:-
      http://mbed.org/users/AjK/libraries/MODDMA/latest
      See example_dma.cpp for more information.
      
1.8 - 22/11/2010

    * Added code so that if a buffer is set to zero length then
      MODSERIAL defaults to just using the FIFO for that stream
      thus making the library "fall back" to teh same operation
      that the Mbed Serial library performs.
    * Removed dmaSend() function that should have been removed 
      at 1.7
    
1.7 - 21/11/2010

    * Remove the DMA enum from MODSERIAL.h as it's not currently 
      ready for release.
    * Added page doxygen comments.

1.6 - 21/11/2010

   * Version 1.5 solved a blocking problem on putc() when called 
     from another ISR. However, isr_tx() invokes a callback of it's
     own when a byte is tranferred from TX buffer to TX FIFO. User
     programs may interpret that as an IRQ callback. That's an ISR
     call from within an existing ISR which is not good. So the 
     TxIrq callback from isr_tx is now conditional. It will only
     be called when isr_tx() is actually within it's own ISR and
     not when called from alternate ISR handlers.
     
1.5 - 21/11/2010

    * Calling putc() (or any derived function that uses it like
      printf()) while inside an interrupt service routine can
      cause the system to lock up if the TX buffer is full. This
      is because bytes are only transferred from the TX buffer to
      the TX FIFO via the TX ISR. If we are, say in an RX ISR already,
      then the TX ISR will never trigger. The TX buffer stays full and
      there is never space to putc() the byte. So, while putc() blocks
      waiting for space it calls isr_tx() to ensure if TX FIFO space
      becomes available it will move bytes from the TX buffer to TX
      FIFO thus removing the blocking condition within putc().

1.4 - 21/11/2010

    * Removed all the new DMA code. I wish mbed.org had proper SVN
      versioning, I'm use to working in HEAD and BRANCHES after I've
      released a project. Getting bug reports in current releases
      while trying to dev new code is hard to manage without source
      control of some type!

1.3 - 21/11/2010

    * Fixed a macro problem with txIsBusy()
    * Started adding code to use "block data" sending using DMA
    * Removed #include "IOMACROS.h"
    
1.2 - 21/11/2010

    * Removed unsed variables from flushBuffer()
    * Fixed a bug where both RX AND TX fifos are cleared/reset 
      when just TX OR RX should be cleared.
    * Fixed a bug that cleared IIR when in fact it should be left
      alone so that any pending interrupt after flush is handled.
    * Merged setBase() into init() as it wasn't required anywhere else.
    * Changed init() to enforce _uidx is set by Serial to define the _base
      address of the Uart in use.
        
1.1 - 20/11/2010

    * Added this file
    * Removed cruft from GETC.cpp
    * "teh" should be "the", why do my fingers do that?

1.0 - 20/11/2010

    * First release.

*/