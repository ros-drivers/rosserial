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

#include "ros.h"
#include "SysTimer.h"

uint32_t SysTimer::millis_;

/*****************************************************************************
* Function Name: SysTick_ISR
******************************************************************************
* Summary: Interrupt on Cortex-M0 SysTick. Toggle LED.
*
* Parameters: none; it's an ISR function
*
* Return: none; it's an ISR function
*
*****************************************************************************/
CY_ISR(SysTick_ISR)
{
    ++SysTimer::millis_;
} ;



