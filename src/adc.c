#include "adc.h"

#include <stdio.h>

#include "stm32g031xx.h"
#include "uart.h"

#define DMAMUX_REQ_ADC1 0x00000005U

#define R1 100000
#define R2 47000

volatile int waiting;
void DMA1_Channel2_3_IRQHandler(void) {
  DMA1->IFCR |= 1 << 5;
  waiting = 0;
}

void adc_read(adc_measurements_t *res) {
  waiting = 1;
  uint32_t data[5] = {0};

  // prepare transfer
  DMA1_Channel2->CNDTR = sizeof(data) / sizeof(data[0]);
  DMA1_Channel2->CMAR = (uint32_t)data;
  DMA1_Channel2->CPAR = (uint32_t)&ADC1->DR;
  DMA1_Channel2->CCR =
      DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | (0b01 << DMA_CCR_PSIZE_Pos) | (0b10 << DMA_CCR_MSIZE_Pos);

  // start conversion
  // ADC1->CR &= ~ADC_CR_BITS_PROPERTY_RS;
  ADC1->CR |= ADC_CR_ADSTART;

  while(waiting);
  // disable DMA
  DMA1_Channel2->CCR = 0;

  // uint16_t vref_real = *((uint16_t *)(0x1FFF75AAUL)) * 3000UL / 4095;

  res->vref_mV = *((uint16_t *)(0x1FFF75AAUL)) * 3000UL / data[3];
  res->bat_I_mV = res->vref_mV * data[0] / 4095;
  res->bat_V_mV = res->vref_mV * data[1] / 4095;
  res->solar_V_mV = res->vref_mV * data[2] / 4095;

  res->bat_I_mV = res->bat_I_mV / (330000 / 47000.0);  //* (47 + 97 + 4.7f) / 47;
  res->bat_V_mV = res->bat_V_mV * (R1 + R2) / R2;
  res->solar_V_mV = res->solar_V_mV * (R1 + R2) / R2;

  res->vbat_mV = data[4] * 2;

  char buf[128];
  // snprintf(buf, sizeof(buf), "vref=%5u VBAT=%5u cal=%5d\r\n", res->vref_mV, res->vbat_mV, *((uint16_t *)(0x1FFF75AAUL)));
  // snprintf(buf, sizeof(buf), "%d %ld\n", vref_real, *((uint16_t *)(0x1FFF75AAUL)) * 3000UL / data[3]);
  uint32_t vref_data = data[3];
  uint32_t vbat_data = data[4];
  uint32_t vrefint = *((uint16_t *)(0x1FFF75AAUL));
  uint16_t vbat = 1000 * 3.0 * ((3.0 * vbat_data * vrefint) / (4095.0 * vref_data));
  snprintf(buf, sizeof(buf), "int=%d vref=%ld vbat=%ld %d\n", *((uint16_t *)(0x1FFF75AAUL)), data[3], data[4], vbat);
  uart_send(buf);

  res->vbat_mV = vbat;
}

void adc_init() {
  RCC->APBENR2 |= RCC_APBENR2_ADCEN;

  // VREFBUF->CSR = 0;
  // while((VREFBUF->CSR & VREFBUF_CSR_VRR_Msk) == 0);

  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

  DMAMUX1_Channel1->CCR = DMAMUX_REQ_ADC1;

  // enable internal 3v3 voltage reference
  ADC1->CR = ADC_CR_ADVREGEN;
  // enable DMA and overwrite data with new newer
  ADC1->CFGR1 = ADC_CFGR1_DMAEN | ADC_CFGR1_OVRMOD;

  // CLK / 2
  ADC1->CFGR2 = 0b01 << ADC_CFGR2_CKMODE_Pos;

  // enable channel 0, 1, voltage reference and VBAT
  ADC1->CHSELR = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 13) | (1 << 14);

  // sampling time
  ADC1->SMPR = 7;

  // enable BAT and VRef channels
  ADC1_COMMON->CCR |= ADC_CCR_VBATEN | ADC_CCR_VREFEN;

  // ENABLE ADC
  ADC1->CR |= ADC_CR_ADEN;
}
