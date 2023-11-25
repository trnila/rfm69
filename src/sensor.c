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
GPIO_TypeDef *irq_port = GPIOB;
const uint32_t irq_pin = 1;

void add_measurement(uint8_t *payload, size_t *offset, uint8_t id, uint32_t value) {
  payload[(*offset)++] = RFM69_CMD_MEASUREMENT;
  payload[(*offset)++] = id;
  payload[(*offset)++] = value;
  payload[(*offset)++] = value >> 8;
  payload[(*offset)++] = value >> 16;
  payload[(*offset)++] = value >> 24;
  assert(*offset < RFM69_FIFO_SIZE);
}

void EXTI0_1_IRQHandler() { EXTI->RPR1 = 1 << irq_pin; }

void main() {
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  timer_init();
  uart_init();
  adc_init();

  RFM69_init(config->radio.node_id);

  // enable IRQ pin
  gpio_input(irq_port, irq_pin);
  gpio_pulldown(irq_port, irq_pin);
  // source is portB
  EXTI->EXTICR[irq_pin / 4] = 0x01 << ((irq_pin & 3) << 3);
  // Rising edge
  EXTI->RTSR1 |= 1U << irq_pin;
  // unask irq - yeah, we are unasking according to docs ^^
  EXTI->IMR1 |= 1U << irq_pin;
  NVIC_EnableIRQ(EXTI0_1_IRQn);

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
        add_measurement(payload, &offset, SENSOR_WINDOW, !gpio_read(irq_port, irq_pin));
        RFM69_send_packet(0, true, offset);
      }
    }
  }
}
