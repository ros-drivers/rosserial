#if defined(TARGET_LPC11U24)

#define MODSERIAL_IRQ_REG ((LPC_USART_Type*)_base)->IER
#define DISABLE_TX_IRQ MODSERIAL_IRQ_REG &= ~(1UL << 1)
#define DISABLE_RX_IRQ MODSERIAL_IRQ_REG &= ~(1UL << 0)
#define ENABLE_TX_IRQ MODSERIAL_IRQ_REG |= (1UL << 1)
#define ENABLE_RX_IRQ MODSERIAL_IRQ_REG |= (1UL << 0)

#define RESET_TX_FIFO ((LPC_USART_Type*)_base)->FCR |= (1UL<<2)
#define RESET_RX_FIFO ((LPC_USART_Type*)_base)->FCR |= (1UL<<1)

#define MODSERIAL_READ_REG ((LPC_USART_Type*)_base)->RBR
#define MODSERIAL_WRITE_REG ((LPC_USART_Type*)_base)->THR
#define MODSERIAL_READABLE ((((LPC_USART_Type*)_base)->LSR & (1UL<<0)) != 0)
#define MODSERIAL_WRITABLE ((((LPC_USART_Type*)_base)->LSR & (1UL<<5)) != 0)

#define RX_IRQ_ENABLED true
#define TX_IRQ_ENABLED true

#endif