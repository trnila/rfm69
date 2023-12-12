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

void wakeup_by_rtc(int seconds) {
  // enable RTC domain
  RCC->APBENR1 |= RCC_APBENR1_RTCAPBEN;

  // Disable RTC domain write protection
  PWR->CR1 |= PWR_CR1_DBP;
  // enable RTC clock and use external low speed crystal 32.768kHz
  RCC->BDCR |= (0b01 << RCC_BDCR_RTCSEL_Pos) | RCC_BDCR_RTCEN | RCC_BDCR_LSEON;

  // unlock RTC registers
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;

  // clear wakeup timer flag
  RTC->SCR |= RTC_SCR_CWUTF;

  // disable wake up timer
  RTC->CR &= ~RTC_CR_WUTE;
  // wait till configuration is allowed
  while(!(RTC->ICSR & RTC_ICSR_WUTWF))
    ;
  // use ck_spre (1hz -> 1 second resolution)
  RTC->CR |= 0b100 << RTC_CR_WUCKSEL_Pos;
  RTC->WUTR = seconds;
  // enable timer
  RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
}

/**
 * Wakeup by rising edge on PA0 and falling edge on PA2
 */
void wakeup_by_pins() {
  // clear all wakeup flags
  PWR->SCR |= PWR_SCR_CWUF;

  // falling edge on wakeup2 pin (PA4), rising edge on wakeup1 pin (PA0)
  PWR->CR4 |= PWR_CR4_WP2;
  // enable wakeup1 and wakeup2
  PWR->CR3 |= PWR_CR3_EWUP1 | PWR_CR3_EWUP2;
}

void deep_sleep() {
  // enter shutdown mode on sleep
  PWR->CR1 |= 0b100 << PWR_CR1_LPMS_Pos;
  // enable deep sleep
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  for(;;) {
    // shutdown - on wakeup CPU resets and starts over
    __WFI();
  }
}

void main() {
  const uint32_t SYSCLK_prescaler = 4;
  const uint32_t SYSCLK = 16000000UL / (1U << SYSCLK_prescaler);
  RCC->CR |= SYSCLK_prescaler << RCC_CR_HSIDIV_Pos;

  // enable PWR domain
  RCC->APBENR1 |= RCC_APBENR1_PWREN;
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

  adc_measurements_t m;
  adc_read(&m);

  uint8_t *payload = RFM69_get_tx_payload();
  size_t offset = 0;
  add_measurement(payload, &offset, SENSOR_BATTERY_CURRENT, m.bat_I_mV);
  add_measurement(payload, &offset, SENSOR_BATTERY_VOLTAGE, m.bat_V_mV);
  add_measurement(payload, &offset, SENSOR_SOLAR_VOLTAGE, m.solar_V_mV);
  add_measurement(payload, &offset, SENSOR_VBAT, m.vbat_mV);
  add_measurement(payload, &offset, SENSOR_VREF, m.vref_mV);
  add_measurement(payload, &offset, SENSOR_WINDOW, !gpio_read(window_port, window_pin));
  RFM69_send_packet(0, true, offset);

  for(;;) {
    if(RFM69_read_packet()) {
      RFM69_rxbuf_return();
      break;
    }
  }

  wakeup_by_pins();
  wakeup_by_rtc(15);
  deep_sleep();
}
