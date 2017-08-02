#include "mkl82z_uart.h"

uint8_t  UART_RX_BUFF[UART_RX_BUFF_LEN];
uint8_t  UART_TX_BUFF[UART_TX_BUFF_LEN];

volatile uint32_t overflow_ms = 0;
volatile uint16_t Rx_UART_HEAD = 0;
volatile uint16_t Rx_UART_TAIL = 0;
volatile uint16_t Tx_UART_HEAD = 0;
volatile uint16_t Tx_UART_TAIL = 0;


void MKL82Z_LPUART_IRQHandler()
{
  uint8_t data;
  uint16_t ii;
  if((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(MKL82Z_LPUART))
  {
    data = MKL82Z_LPUART->DATA;

    ii = (uint16_t) (Rx_UART_HEAD+1)%UART_RX_BUFF_LEN;
    if(ii != Rx_UART_TAIL )
    {
      UART_RX_BUFF[Rx_UART_HEAD] = data;
      Rx_UART_HEAD = ii;
    }
  }
  else if(kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(MKL82Z_LPUART))
  {
    data = UART_TX_BUFF[Tx_UART_TAIL];
    Tx_UART_TAIL = (Tx_UART_TAIL+1) % UART_TX_BUFF_LEN;
    LPUART_WriteByte(MKL82Z_LPUART,data);
    if(Tx_UART_TAIL == Tx_UART_HEAD )
    {
      LPUART_DisableInterrupts(MKL82Z_LPUART,kLPUART_TxDataRegEmptyInterruptEnable);
    }
  }
}

void MKL82Z_PIT0_IRQHandler()
{
  PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
  overflow_ms++;
}

void MKL82Z_PIT_init()
{
  pit_config_t pitConfig;
  PIT_GetDefaultConfig(&pitConfig);
  PIT_Init(PIT, &pitConfig);
  PIT_SetTimerPeriod(PIT,kPIT_Chnl_0,USEC_TO_COUNT(1000U, MKL82Z_PIT_SRC_CLK));
  PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
  EnableIRQ(MKL82Z_IRQ_ID);
  PIT_StartTimer(PIT, kPIT_Chnl_0);
}

void MKL82Z_UART_init()
{
  int ii;
  lpuart_config_t config;
  CLOCK_SetLpuartClock(1U);
  LPUART_GetDefaultConfig(&config);
  config.baudRate_Bps = 57600;
  config.enableTx = true;
  config.enableRx = true;
  for(ii=0;ii<30000;ii++)
  {
    __asm("NOP");
  }
  LPUART_Init(MKL82Z_LPUART, &config, CLOCK_GetFreq(MKL82Z_CLK_SEL));
  LPUART_DisableInterrupts(MKL82Z_LPUART,kLPUART_TxDataRegEmptyInterruptEnable);
  LPUART_EnableInterrupts(MKL82Z_LPUART,kLPUART_RxDataRegFullInterruptEnable);
  EnableIRQ(MKL82Z_LPUART_IRQn);
}

int16_t MKL82Z_UART_recbyte()
{
  uint8_t data;
  if(Rx_UART_HEAD == Rx_UART_TAIL)
  {
    return -1;
  }
  else
  {
    data = UART_RX_BUFF[Rx_UART_TAIL];
    Rx_UART_TAIL = (uint16_t) (Rx_UART_TAIL+1) %UART_RX_BUFF_LEN;
    return data;
  }
}

int MKL82Z_UART_write_byte(uint8_t data)
{
  uint16_t ii;
  if((Tx_UART_HEAD==Tx_UART_TAIL)&&(kLPUART_TxDataRegEmptyFlag & LPUART_GetStatusFlags(MKL82Z_LPUART)))
  {
    LPUART_WriteByte(MKL82Z_LPUART,data);
    return 1;
  }
  ii = (Tx_UART_HEAD+1) % UART_TX_BUFF_LEN;
  while (ii==Tx_UART_TAIL)
  {
    __asm("NOP");
  }
  UART_TX_BUFF[Tx_UART_HEAD] = data;
  Tx_UART_HEAD = ii;
  LPUART_EnableInterrupts(MKL82Z_LPUART,kLPUART_TxDataRegEmptyInterruptEnable);
  return 1;
}

uint32_t MKL82Z_time()
{
  uint32_t now;
  PIT_DisableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
  now = overflow_ms;
  PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
  return now;
}
