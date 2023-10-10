extern void _estack(void);    // top of the stack, function so it can be placed in vector table
extern unsigned int _sidata;  // .data in flash
extern unsigned int _sdata;   // start of .data in SRAM
extern unsigned int _edata;   // end of .data in SRAM
extern unsigned int _sbss;    // start of .bss in SRAM
extern unsigned int _ebss;    // end of .bss in SRAM

void main();

void Reset_Handler() {
  // copy .data section from flash to sram
  const unsigned int *src = &_sidata;
  unsigned int *dst = &_sdata;
  while(dst < &_edata) {
    *dst++ = *src++;
  }

  // zero the .bss section
  dst = &_sbss;
  while(dst < &_ebss) {
    *dst++ = 0;
  }

  main();
  for(;;)
    ;
}

static void default_irq_handler() {
  for(;;) {
    asm("bkpt #1");
  }
}

__attribute__((weak, alias("default_irq_handler"))) void NMI_Handler();
__attribute__((weak, alias("default_irq_handler"))) void HardFault_Handler();
__attribute__((weak, alias("default_irq_handler"))) void SVC_Handler();
__attribute__((weak, alias("default_irq_handler"))) void PendSV_Handler();
__attribute__((weak, alias("default_irq_handler"))) void SysTick_Handler();
__attribute__((weak, alias("default_irq_handler"))) void WWDG_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void PVD_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void RTC_TAMP_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void FLASH_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void RCC_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void EXTI0_1_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void EXTI2_3_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void EXTI4_15_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void DMA1_Channel1_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void DMA1_Channel2_3_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void ADC1_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void TIM1_BRK_UP_TRG_COM_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void TIM1_CC_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void TIM2_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void TIM3_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void LPTIM1_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void LPTIM2_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void TIM14_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void TIM16_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void TIM17_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void I2C1_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void I2C2_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void SPI1_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void SPI2_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void USART1_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void USART2_IRQHandler();
__attribute__((weak, alias("default_irq_handler"))) void LPUART1_IRQHandler();

__attribute__((used, section(".isr_vector"))) void (*vectors[])(void) = {
    &_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SVC_Handler,
    0,
    0,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,
    PVD_IRQHandler,
    RTC_TAMP_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_1_IRQHandler,
    EXTI2_3_IRQHandler,
    EXTI4_15_IRQHandler,
    0,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_3_IRQHandler,
    DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler,
    ADC1_IRQHandler,
    TIM1_BRK_UP_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    LPTIM1_IRQHandler,
    LPTIM2_IRQHandler,
    TIM14_IRQHandler,
    0,
    TIM16_IRQHandler,
    TIM17_IRQHandler,
    I2C1_IRQHandler,
    I2C2_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    LPUART1_IRQHandler,
    0,
};
