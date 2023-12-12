#include <stdio.h>
#include <string.h>

#include "RFM69.h"
#include "adc.h"
#include "gpio.h"
#include "sensors.h"
#include "stm32g031xx.h"
#include "timer.h"
#include "uart.h"

typedef struct {
  uint8_t node_id;
  uint8_t key[8];
} Radio;

typedef struct {
  Radio radio;
} Config;

const Config *config = (const Config *)0x0800F800;
extern RFM69_Packet *tx_packet;
GPIO_TypeDef *window_port = GPIOA;
const uint32_t window_pin = 0;

void add_measurement(uint8_t *payload, size_t *offset, uint8_t id, uint32_t value) {
  payload[(*offset)++] = RFM69_CMD_MEASUREMENT;
  payload[(*offset)++] = id;
  payload[(*offset)++] = value;
  payload[(*offset)++] = value >> 8;
  payload[(*offset)++] = value >> 16;
  payload[(*offset)++] = value >> 24;
  assert(*offset < RFM69_FIFO_SIZE);
}

void main() {
  const uint32_t SYSCLK_prescaler = 4;
  const uint32_t SYSCLK = 16000000UL / (1U << SYSCLK_prescaler);
  RCC->CR |= SYSCLK_prescaler << RCC_CR_HSIDIV_Pos;

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  timer_init(SYSCLK);
  uart_init(SYSCLK, 9600U);
  adc_init();

  RFM69_init(config->radio.node_id);

  // configure window pin as an input
  gpio_input(window_port, window_pin);
  gpio_pulldown(window_port, window_pin);

  uint32_t last_measurement_ms = ((uint32_t)-1) / 2;
  for(;;) {
    if(RFM69_read_packet()) {
      RFM69_rxbuf_return();
    }

    if((tick_ms - last_measurement_ms) > 1000) {
      last_measurement_ms = tick_ms;
      adc_measurements_t m;
      adc_read(&m);

      uint8_t *payload = RFM69_get_tx_payload();
      if(payload) {
        size_t offset = 0;
        add_measurement(payload, &offset, SENSOR_BATTERY_CURRENT, m.bat_I_mV);
        add_measurement(payload, &offset, SENSOR_BATTERY_VOLTAGE, m.bat_V_mV);
        add_measurement(payload, &offset, SENSOR_SOLAR_VOLTAGE, m.solar_V_mV);
        add_measurement(payload, &offset, SENSOR_VBAT, m.vbat_mV);
        add_measurement(payload, &offset, SENSOR_VREF, m.vref_mV);
        add_measurement(payload, &offset, SENSOR_WINDOW, !gpio_read(window_port, window_pin));
        RFM69_send_packet(0, true, offset);
      }
    }
  }
}
