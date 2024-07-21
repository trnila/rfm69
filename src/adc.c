#include "adc.h"

#include <stdio.h>

#include "stm32g031xx.h"
#include "uart.h"

#define DMAMUX_REQ_ADC1 0x00000005U

volatile int waiting;
void DMA1_Channel2_3_IRQHandler(void) {
  DMA1->IFCR |= 1 << 5;
  waiting = 0;
}

void adc_read(adc_measurements_t *res) {
  waiting = 1;
  uint32_t data[3] = {0};

  // prepare transfer
  DMA1_Channel2->CNDTR = sizeof(data) / sizeof(data[0]);
  DMA1_Channel2->CMAR = (uint32_t)data;
  DMA1_Channel2->CPAR = (uint32_t)&ADC1->DR;
  DMA1_Channel2->CCR =
      DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | (0b01 << DMA_CCR_PSIZE_Pos) | (0b10 << DMA_CCR_MSIZE_Pos);

  // start conversion
  ADC1->CR |= ADC_CR_ADSTART;

  while(waiting);
  // disable DMA
  DMA1_Channel2->CCR = 0;

  char buf[128];
  uint32_t vref_data = data[1];
  uint32_t vbat_data = data[2];
  uint32_t vrefint = *((uint16_t *)(0x1FFF75AAUL));
  uint16_t vbat = 1000 * 3.0 * ((3.0 * vbat_data * vrefint) / (4095.0 * vref_data));

  float TS_CAL1_TEMP = 30;
  float TS_CAL1 = *(uint16_t *)0x1FFF75A8UL;
  float TS_CAL2_TEMP = 130;
  float TS_CAL2 = *(uint16_t *)0x1FFF75CAUL;
  uint16_t temp = 10.0 * ((TS_CAL2_TEMP - TS_CAL1_TEMP) / (TS_CAL2 - TS_CAL1) * (data[0] - TS_CAL1) + TS_CAL1_TEMP);
  snprintf(buf, sizeof(buf), "%d\n", temp);

  snprintf(buf, sizeof(buf), "int=%d vref=%ld vbat=%ld %d temp=%d\n", *((uint16_t *)(0x1FFF75AAUL)), vref_data, vbat_data, vbat, temp);

  uart_send(buf);

  res->vbat_mV = vbat;
}

void adc_init() {
  RCC->APBENR2 |= RCC_APBENR2_ADCEN;

  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  DMAMUX1_Channel1->CCR = DMAMUX_REQ_ADC1;

  // enable internal 3v3 voltage reference
  ADC1->CR = ADC_CR_ADVREGEN;
  // enable DMA and overwrite data with new newer
  ADC1->CFGR1 = ADC_CFGR1_DMAEN | ADC_CFGR1_OVRMOD;

  // CLK / 2
  ADC1->CFGR2 = 0b01 << ADC_CFGR2_CKMODE_Pos;

  // enable channel Vsense, voltage reference and VBAT
  ADC1->CHSELR = (1 << 12) | (1 << 13) | (1 << 14);

  // sampling time
  ADC1->SMPR = 7;

  // enable BAT and VRef channels
  ADC1_COMMON->CCR |= ADC_CCR_TSEN | ADC_CCR_VBATEN | ADC_CCR_VREFEN;

  // ENABLE ADC
  ADC1->CR |= ADC_CR_ADEN;
}
