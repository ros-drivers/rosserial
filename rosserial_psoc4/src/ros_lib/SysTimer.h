/* ========================================
 *
 * The following firmware was developed by Chuck Harrison
 * This work is licensed under a Creative Commons Attribution 3.0 Unported License.
 * 
 * http://creativecommons.org/licenses/by/3.0/deed.en_US
 * 
 * You are free to:
 * -To Share — to copy, distribute and transmit the work 
 * -To Remix — to adapt the work 
 * -To make commercial use of the work
 *
 * ========================================
 */

#ifndef ROS_SYSTIMER_H_
#define ROS_SYSTIMER_H_

typedef void (* cyisraddress)(void); // should we #include the whole cytypes.h?
#define CY_ISR(FuncName)        void FuncName (void)

extern "C" {
cyisraddress CyIntSetSysVector(uint8_t number, cyisraddress address);
}

CY_ISR(SysTick_ISR); //forward declaration for SysTimer::init()

#define SYSTICK_INTERRUPT_VECTOR_NUMBER 15u /* Cortex-M0 hard vector */

/* clock and interrupt rates, in Hz */
#define CLOCK_FREQ     24000000u
#define INTERRUPT_FREQ 1000u


class SysTimer
{
  public:
    static void init() {
      CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, SysTick_ISR);
      // must eventually do this:
      //  SysTick_Config(CLOCK_FREQ / INTERRUPT_FREQ); /* defined in auto-generated core_cm0.h */
      //  CyGlobalIntEnable;
      millis_ = 0;
    };
    static uint32_t millis() { return millis_; };

    static uint32_t millis_;
} ;

#endif

