#include "gpio.h"
#include "stm32g031xx.h"

#define GPIO_AF1_USART2 1

void uart_init(uint32_t clk_in, uint32_t bauds) {
  /**USART2 GPIO Configuration
  PA2     ------> USART2_TX
  PA3     ------> USART2_RX
  */
  gpio_alt(GPIOA, 2, GPIO_AF1_USART2);
  gpio_alt(GPIOA, 3, GPIO_AF1_USART2);

  RCC->APBENR1 |= RCC_APBENR1_USART2EN;

  USART2->BRR = (clk_in + bauds / 2) / bauds;
  USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
  USART2->CR2 = 0;
  USART2->CR3 = 0;
}

void uart_send(const char *str) {
  while(*str != '\0') {
    while(!(USART2->ISR & USART_ISR_TXE_TXFNF))
      ;
    USART2->TDR = *str;
    str++;
  }
}
