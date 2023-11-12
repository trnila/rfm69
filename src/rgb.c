#include "rgb.h"

#include <stdbool.h>
#include <string.h>

#include "gpio.h"
#include "stm32g031xx.h"

#define GPIO_AF2_TIM1 2
#define NUM_PIXELS 23

static uint8_t data[NUM_PIXELS * 3 * 8];

static uint8_t HIGH;
static uint8_t LOW;
static uint8_t brightness = 255;
static volatile bool updated = 0;

void rgb_set_brightness(uint8_t br) {
  brightness = br;
  updated = true;
}

void rgb_set(size_t pos, uint8_t r, uint8_t g, uint8_t b) {
  size_t offset = pos * 3 * 8;
  uint8_t rgb[] = {(unsigned int)g * brightness / 255U, (unsigned int)r * brightness / 255U,
                   (unsigned int)b * brightness / 255U};
  for(int c = 0; c < 3; c++) {
    for(int bit = 7; bit; bit--) {
      data[offset++] = (rgb[c] & (1U << bit)) ? HIGH : LOW;
    }
  }

  updated = true;
}

void rgb_clear() {
  memset(data, LOW, sizeof(data));
  updated = true;
}

void rgb_update() {
  if(!updated || (TIM1->DIER & TIM_DIER_CC1DE)) {
    return;
  }
  updated = false;

  TIM1->BDTR |= TIM_BDTR_MOE;

  TIM1->CCER |= TIM_CCER_CC1E;

  TIM1->CR1 |= TIM_CR1_CEN;
  DMA1_Channel1->CNDTR = sizeof(data);
  DMA1_Channel1->CMAR = (uint32_t)data;
  DMA1_Channel1->CPAR = (uint32_t)&TIM1->CCR1;
  DMA1_Channel1->CCR = DMA_CCR_EN | DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC |
                       (0b01 << DMA_CCR_PSIZE_Pos) | (0b11 << DMA_CCR_PL_Pos);

  TIM1->DIER |= TIM_DIER_CC1DE;
}

void rgb_init(void) {
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  // enable clock
  RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
  // route dma
  DMAMUX1_Channel0->CCR = 0x14;
  // enable PWM output
  TIM1->CCMR1 = (0b110 << TIM_CCMR1_IC1F_Pos) | TIM_CCMR1_OC1PE;

  gpio_alt(GPIOA, 8, GPIO_AF2_TIM1);
  gpio_speed(GPIOA, 8, 0b11);

  //(__LL_RCC_CALC_PCLK1_FREQ(HAL_RCC_GetHCLKFreq(),
  // LL_RCC_GetAPB1Prescaler())));
  uint32_t freq = 16000000 / (400 * 1000);

  HIGH = freq * 0.48 - 1;
  LOW = freq * 0.20 - 1;

  TIM1->PSC = 0;
  TIM1->ARR = freq - 1;
  TIM1->EGR = 1;

  rgb_clear();
}

void DMA1_Channel1_IRQHandler(void) {
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  // DMA1->IFCR |= 1 DMA_IFCR_CGIF1;
  //  clear irq
  DMA1->IFCR = 7;
  DMA1_Channel1->CCR = 0;
  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->DIER &= ~TIM_DIER_CC1DE;
}
