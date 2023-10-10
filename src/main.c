#include <stdio.h>

#include "adc.h"
#include "gpio.h"
#include "stm32g031xx.h"
#include "uart.h"

uint32_t SystemCoreClock = 16000000UL;

int main(void) {
  // calibrace
  RCC->ICSCR &= ~RCC_ICSCR_HSITRIM;
  RCC->ICSCR |= 64 << RCC_ICSCR_HSITRIM_Pos;

  // delicka
  // MODIFY_REG(RCC->CR, RCC_CR_HSIDIV, (__HSIDIV__))

  // SystemCoreClock = (HSI_VALUE / (1UL << ((READ_BIT(RCC->CR, RCC_CR_HSIDIV))
  // >> RCC_CR_HSIDIV_Pos)));
  SystemCoreClock = 16000000U;

  // APB prescaler 11
  // RCC->CFGR |= 0b111 << RCC_CFGR_PPRE_Pos;

  // SystemCoreClock = (HAL_RCC_GetSysClockFreq() >> ((AHBPrescTable[(RCC->CFGR
  // & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos]) & 0x1FU));

  /* GPIO Ports Clock Enable */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN;

  gpio_output(GPIOC, 6, 0);

  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  uart_init();

  adc_init();

  uart_send("Init\r\n");

  void app_main();
  app_main();
}
