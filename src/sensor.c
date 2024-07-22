#include <stdio.h>
#include <string.h>

#include "RFM69.h"
#include "adc.h"
#include "config.h"
#include "gpio.h"
#include "protocol.h"
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
GPIO_TypeDef *window_port = GPIOA;
const uint32_t window_pin = 0;

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
  while(!(RTC->ICSR & RTC_ICSR_WUTWF));
  // use ck_spre (1hz -> 1 second resolution)
  RTC->CR |= 0b100 << RTC_CR_WUCKSEL_Pos;
  RTC->WUTR = seconds;
  // enable timer
  RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
}

/**
 * Wakeup by gpio level on WKPUP1 (PA0)
 */
void wakeup_by_pins(int level) {
  // clear all wakeup flags
  PWR->SCR |= PWR_SCR_CWUF;

  if(!level) {
    PWR->CR4 |= PWR_CR4_WP1;
  }
  // enable WKUP1
  PWR->CR3 |= PWR_CR3_EWUP1;
}

[[noreturn]] void deep_sleep() {
  // enter shutdown mode on sleep
  PWR->CR1 |= 0b100 << PWR_CR1_LPMS_Pos;
  // enable deep sleep
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  for(;;) {
    // shutdown - on wakeup CPU resets and starts over
    __WFI();
  }
}

[[noreturn]] void main() {
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
  // use pull-up for PA0 in shutdown
  PWR->CR3 |= PWR_CR3_APC;
  PWR->PUCRA |= 1 << 0;

  uint8_t cnt = 0;
  for(;;) {
    adc_measurements_t m;
    adc_read(&m);

    bool window_level = gpio_read(window_port, window_pin);
    struct SensorState *payload = (struct SensorState *)RFM69_get_tx_payload();
    payload->open = !window_level;
    payload->voltage = m.vbat_mV;
    payload->firmware = 0x42;
    extern uint8_t RSSI;
    payload->RSSI = RSSI;
    payload->counter = cnt++;
    RFM69_send_packet(0, true, sizeof(*payload));

    RFM69_Packet *packet;
    uint32_t timeout_ms = tick_ms + 5000;
    while((packet = RFM69_read_packet()) == NULL && tick_ms < timeout_ms);

    if(packet) {
      struct SensorStateAck *ack = (struct SensorStateAck *)&packet->payload;
      char buf[64];
      snprintf(buf, sizeof(buf), "ACK len=%x dst=%x src=%x state=%d fw=%x\n", packet->hdr.length, packet->hdr.dst, packet->hdr.src, ack->open, ack->fw);
      uart_send(buf);
      for(uint32_t i = 0; i < 1000; i++) {
        asm("nop");
      }
    }

    for(uint32_t i = 0; i < 100000; i++) {
      asm("nop");
    }
    uart_send("cycle\n");
  }

  // wakeup_by_pins(!window_level);
  // wakeup_by_rtc(KEEPALIVE_MS / 1000);
  // deep_sleep();
}
