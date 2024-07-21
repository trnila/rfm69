#pragma once

#include <stdint.h>

typedef struct {
  uint16_t vbat_mV;
} adc_measurements_t;

void adc_init();
void adc_read(adc_measurements_t *res);
