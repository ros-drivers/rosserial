
#include "ros.h"
#include "SysTimer.h"
#include "device.h"


void init(void)
{
  SysTimer::init();
  SysTick_Config(CLOCK_FREQ / INTERRUPT_FREQ); /* defined in auto-generated core_cm0.h */

  CyGlobalIntEnable;
}
