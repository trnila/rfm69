#include "RFM69.h"

#include "gpio.h"
#include "stm32g031xx.h"

#define GPIO_AF0_SPI1 0x00

#define CS_PORT GPIOA
#define CS_PIN 15

#define RFM69_FIFO_SIZE 66

static volatile bool received = false;
static uint8_t data[RFM69_FIFO_SIZE];
static const uint32_t irq_pin = 6;

void spi_init() {
  RCC->APBENR2 |= RCC_APBENR2_SPI1EN;

  // SCK
  gpio_alt(GPIOB, 3, GPIO_AF0_SPI1);
  // MISO
  gpio_alt(GPIOB, 4, GPIO_AF0_SPI1);
  // MOSI
  gpio_alt(GPIOB, 5, GPIO_AF0_SPI1);
  // CS
  gpio_output(CS_PORT, CS_PIN, 1);

  SPI1->CR1 = SPI_CR1_SPE | SPI_CR1_MSTR | (0b100 << SPI_CR1_BR_Pos) | SPI_CR1_SSI | SPI_CR1_SSM;
  SPI1->CR2 = SPI_CR2_FRXTH;
}

static uint8_t RFM69_transmit(uint8_t addr, uint8_t val) {
  const uint8_t tx[] = {addr, val};
  uint8_t rx[2];

  CS_PORT->BRR = 1U << CS_PIN;

  SPI1->CR1 |= SPI_CR1_SPE;

  for(int i = 0; i < 2; i++) {
    while((SPI1->SR & SPI_SR_TXE) == 0)
      ;
    *(volatile uint8_t *)&SPI1->DR = tx[i];

    while((SPI1->SR & SPI_SR_RXNE) == 0)
      ;
    rx[i] = SPI1->DR;
  }

  CS_PORT->BSRR = 1U << CS_PIN;

  return rx[1];
}

void RFM69_write(uint8_t addr, uint8_t val) { RFM69_transmit(addr | 0b10000000, val); }

void RFM69_write16(uint8_t addr, uint16_t val) {
  RFM69_write(addr, val >> 8);
  RFM69_write(addr + 1, val);
}

uint8_t RFM69_read(uint8_t addr) { return RFM69_transmit(addr, 0); }

void RFM69_set_mode(RFM69_Mode mode) {
  RFM69_write(1, mode << 2);

  // mode ready
  while(!(RFM69_read(RFM69_REG_IRQFLAGS1) & (1 << 7)))
    ;
}

void EXTI4_15_IRQHandler() {
  received = 1;
  EXTI->RPR1 = 1 << irq_pin;
}

uint8_t *RFM69_readmsg(size_t *len) {
  if(!received) {
    return NULL;
  }

  received = 0;

  *len = RFM69_read(RFM69_REG_FIFO);
  if(*len >= sizeof(data)) {
    *len = sizeof(data) - 1;
  }
  data[*len] = 0;

  for(uint8_t i = 0; i < *len; i++) {
    data[i] = RFM69_read(RFM69_REG_FIFO);
  }

  return data;
}

void RFM69_init() {
  spi_init();

  // wait until it boots and return version
  while(RFM69_read(RFM69_REG_VERSION) != 0x24)
    ;

  const uint32_t bitrate = 4800;
  const uint32_t F_dev = 50000;

  const uint32_t F_osc = 32000000;
  const uint32_t F_step = F_osc / 524288;  // 524288 = 2**19
  const uint32_t networkID = 0xABCDEF;

  RFM69_set_mode(RFM69_MODE_SLEEP);
  // 868Mhz
  // Frf / (Fxosc / 2**19)
  // 868000000 / (32000000 / 2**19)
  RFM69_write(RFM69_REG_FRFMSB, 0xD9);
  RFM69_write(RFM69_REG_FRFMID, 0x00);
  RFM69_write(RFM69_REG_FRFLSB, 0x00);

  RFM69_write16(RFM69_REG_BITRATEMSB, F_osc / bitrate);
  RFM69_write16(RFM69_REG_FDEVMSB, F_dev / F_step);
  RFM69_write(RFM69_REG_RXBW, 0x42);

  RFM69_write(RFM69_REG_DIOMAPPING2, 0x07);

  // two sync bytes
  RFM69_write(RFM69_REG_SYNCCONFIG, 0x88);
  RFM69_write16(RFM69_REG_SYNCVALUE1, networkID & 0xFFFFU);

  // variable packet
  RFM69_write(RFM69_REG_PACKETCONFIG1, 0b10010000);
  RFM69_write(RFM69_REG_PAYLOADLENGTH, 1);
  RFM69_write(RFM69_REG_PAYLOADLENGTH, 66);

  // start sending once fifo has at least one byte
  RFM69_write(RFM69_REG_FIFOTHRESH, 0x80);

  // enable IRQ
  // Rx - PayloadReady
  // Tx - TxReady
  RFM69_write(RFM69_REG_DIOMAPPING1, 0b01 << 6);

  // enable IRQ pin
  gpio_input(GPIOB, irq_pin);
  // source is portB
  EXTI->EXTICR[irq_pin / 4] = 0x01 << ((irq_pin & 3) << 3);
  // Rising edge
  EXTI->RTSR1 |= 1U << irq_pin;
  // unask irq - yeah, we are unasking according to docs ^^
  EXTI->IMR1 = 1U << irq_pin;
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}
