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
GPIO_TypeDef *irq_port = GPIOA;
const uint32_t irq_pin = 0;

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
  RCC->APBENR1 |= RCC_APBENR1_PWREN;
  RCC->APBENR1 |= RCC_APBENR1_RTCAPBEN;
  PWR->CR1 |= PWR_CR1_DBP;
  RCC->BDCR |= (0b01 << RCC_BDCR_RTCSEL_Pos) | RCC_BDCR_RTCEN | RCC_BDCR_LSEON;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->SCR |= RTC_SCR_CWUTF;

  timer_init(SYSCLK);
  uart_init();
  // adc_init();

  RFM69_init(config->radio.node_id);

  // enable IRQ pin
  gpio_input(irq_port, irq_pin);
  gpio_pulldown(irq_port, irq_pin);

  // adc_measurements_t m;
  // adc_read(&m);

  uint8_t *payload = RFM69_get_tx_payload();
  if(payload) {
    size_t offset = 0;
    // add_measurement(payload, &offset, SENSOR_BATTERY_CURRENT, m.bat_I_mV);
    // add_measurement(payload, &offset, SENSOR_BATTERY_VOLTAGE, m.bat_V_mV);
    // add_measurement(payload, &offset, SENSOR_SOLAR_VOLTAGE, m.solar_V_mV);
    // add_measurement(payload, &offset, SENSOR_VBAT, m.vbat_mV);
    // add_measurement(payload, &offset, SENSOR_VREF, m.vref_mV);
    add_measurement(payload, &offset, SENSOR_WINDOW, !gpio_read(irq_port, irq_pin));
    RFM69_send_packet(0, true, offset);
  }

  while(RFM69_get_tx_payload() == NULL)
    ;

  RTC->CR &= ~RTC_CR_WUTE;
  while(!(RTC->ICSR & RTC_ICSR_WUTWF))
    ;
  RTC->CR |= 0b100 << RTC_CR_WUCKSEL_Pos;
  RTC->WUTR = 5;
  RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;

  PWR->CR3 |= PWR_CR3_EWUP1;

  PWR->SCR |= PWR_SCR_CWUF;
  // enter shutdown mode
  PWR->CR1 |= 0b100 << PWR_CR1_LPMS_Pos;
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  for(;;) {
    __WFI();
  }
}
