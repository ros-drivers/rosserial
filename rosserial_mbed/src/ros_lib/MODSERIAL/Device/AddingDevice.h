/*

To add another device some functions/macros need to be defined.
The Device folder shows examples of other devices.

1. In MACROS.h the include to the macro file need to be added.

2. A <NameOfYourFile.h> file needs to be added, it needs to include the ifdef(target) statement, and definitions for:
MODSERIAL_IRQ_REG    ---   register which enables/disables serial IRQs
DISABLE_TX_IRQ       ---   macro that disables TX IRQs
DISABLE_RX_IRQ       ---   macro that disables RX IRQs
ENABLE_TX_IRQ        ---   macro that enables TX IRQs
ENABLE_RX_IRQ        ---   macro that enables RX IRQs

RESET_TX_FIFO        ---   macro that resets TX FIFO buffer, if applicable. If not, something like while(1==0) won't do anything and doesn't generate compiler warnings
RESET_RX_FIFO        ---   macro that resets RX FIFO buffer, if no hardware options, you can also read data until no more data is available

MODSERIAL_READ_REG   ---   register where RX data is in
MODSERIAL_WRITE_REG  ---   register where TX data is in (can be same as previous)
MODSERIAL_READABLE   ---   returns true if new data is available in read_reg
MODSERIAL_WRITABLE   ---   returns true if we may write new data in write_reg

RX_IRQ_ENABLED       ---   checks if RX IRQs are enabled by MODSERIAL. Only required if the device has no registers that tell which IRQ fired. If those registers are available (LPCs have then), may just be 'true'
TX_IRQ_ENABLED       ---   checks if TX IRQs are enabled by MODSERIAL. See previous

3. A <NameOfYourFile.cpp> file needs to be added, it needs to include the ifdef(target) statement, and functions for:
void setBase(void)   ---   function that sets _base pointer to point to correct UART, and _IRQ pointer to point to correct IRQ.

void initDevice(void)---   function that allows for setting registers after everything else is initialized, if required

bool txIsBusy(void)  ---   function that returns true as long as long as device isn't done with transmitting

*/
 