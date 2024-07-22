#include "RFM69.h"

#include <assert.h>

#include "gpio.h"
#include "stm32g031xx.h"
#include "timer.h"

static_assert(sizeof(RFM69_Header) == 4);

#define RFM69_REG_FIFO 0x00
#define RFM69_REG_OPMODE 0x01
#define RFM69_REG_BITRATEMSB 0x03
#define RFM69_REG_FDEVMSB 0x05
#define RFM69_REG_FRFMSB 0x07
#define RFM69_REG_FRFMID 0x08
#define RFM69_REG_FRFLSB 0x09
#define RFM69_REG_RXBW 0x19
#define RFM69_REG_DIOMAPPING1 0x25
#define RFM69_REG_DIOMAPPING2 0x26
#define RFM69_REG_SYNCCONFIG 0x2E
#define RFM69_REG_SYNCVALUE1 0x2F
#define RFM69_REG_PACKETCONFIG1 0x37
#define RFM69_REG_PAYLOADLENGTH 0x38
#define RFM69_REG_NODEADRS 0x39
#define RFM69_REG_FIFOTHRESH 0x3C

#define RFM69_WRITE_MASK 0b10000000

#define GPIO_AF0_SPI1 0x00

#define CS_PORT GPIOA
#define CS_PIN 15

typedef struct __attribute__((packed)) {
  uint8_t reg;
  RFM69_Packet packet;
} DMABuffer;

static GPIO_TypeDef *TX_LED_PORT = GPIOB;
static unsigned int TX_LED_PIN = 1;
static GPIO_TypeDef *RX_LED_PORT = GPIOA;
static unsigned int RX_LED_PIN = 10;
static const uint32_t irq_pin = 6;

static timer *rx_led_timer;
static timer *tx_led_timer;

static DMABuffer rx_buf;

static DMABuffer tx_buf;

static bool is_gateway;

static volatile bool spi_finished = true;
static uint8_t tx_spi_buf[2];
static uint8_t rx_spi_buf[2];

uint8_t RSSI;

void process_state_machine();

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
  SPI1->CR2 = SPI_CR2_FRXTH | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

  // MUX TX DMA request
  DMAMUX1_Channel3->CCR = 17;
  // MUX RX DMA request
  DMAMUX1_Channel4->CCR = 16;
  NVIC_EnableIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);
}

void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() {
  DMA1->IFCR = 1 << (4 * 4 + 1);
  DMA1_Channel4->CCR = 0;
  DMA1_Channel5->CCR = 0;
  spi_finished = true;

  CS_PORT->BSRR = 1U << CS_PIN;
}

void spi_transmit(const uint8_t *tx, uint8_t *rx, size_t len) {
  assert(spi_finished == 1);
  spi_finished = false;

  CS_PORT->BRR = 1U << CS_PIN;

  __DMB();

  unsigned int tx_flags = DMA_CCR_MINC;
  unsigned int rx_flags = DMA_CCR_MINC;

  if(!tx) {
    tx_spi_buf[0] = 0;
    tx = tx_spi_buf;
    tx_flags = 0;
  }

  if(!rx) {
    rx = rx_spi_buf;
    rx_flags = 0;
  }

  // TX
  DMA1_Channel4->CNDTR = len;
  DMA1_Channel4->CMAR = (uint32_t)tx;
  DMA1_Channel4->CPAR = (uint32_t)&SPI1->DR;
  DMA1_Channel4->CCR = DMA_CCR_EN | DMA_CCR_DIR | tx_flags;

  // RX
  DMA1_Channel5->CNDTR = len;
  DMA1_Channel5->CMAR = (uint32_t)rx;
  DMA1_Channel5->CPAR = (uint32_t)&SPI1->DR;
  DMA1_Channel5->CCR = DMA_CCR_EN | DMA_CCR_TCIE | rx_flags;
}

static uint8_t RFM69_transmit_blocking(uint8_t addr, uint8_t val) {
  const uint8_t tx[] = {addr, val};
  uint8_t rx[] = {0, 0};

  spi_transmit(tx, rx, sizeof(tx));
  while(spi_finished == false);
  __DMB();

  return rx[1];
}

void RFM69_write_blocking(uint8_t addr, uint8_t val) { RFM69_transmit_blocking(addr | RFM69_WRITE_MASK, val); }

void RFM69_write16(uint8_t addr, uint16_t val) {
  RFM69_write_blocking(addr, val >> 8);
  RFM69_write_blocking(addr + 1, val);
}

uint8_t RFM69_read_blocking(uint8_t addr) { return RFM69_transmit_blocking(addr, 0); }

void rx_led_off(timer *t) { gpio_output(RX_LED_PORT, RX_LED_PIN, 0); }

void tx_led_off(timer *t) { gpio_output(TX_LED_PORT, TX_LED_PIN, 0); }

RFM69_Packet *RFM69_read_packet() {
  RFM69_write_blocking(RFM69_REG_OPMODE, 0b100 << 2);

  if(gpio_read(GPIOB, irq_pin) == 0) {
    return NULL;
  }

  RSSI = RFM69_read_blocking(0x24);

  uint8_t *p = (uint8_t *)&rx_buf.packet;

  int len = RFM69_read_blocking(RFM69_REG_FIFO);
  p[0] = len;
  for(int i = 0; i < len; i++) {
    p[i + 1] = RFM69_read_blocking(RFM69_REG_FIFO);
  }

  return &rx_buf.packet;
}

uint8_t *RFM69_get_tx_payload() {
  return tx_buf.packet.payload;
}

void RFM69_send_packet(uint8_t dst, bool require_ack, uint8_t payload_len) {
  tx_buf.packet.hdr.length = payload_len + sizeof(tx_buf.packet.hdr) - 1;
  tx_buf.packet.hdr.dst = dst;
  tx_buf.packet.hdr.flags.seq++;
  tx_buf.packet.hdr.flags.req_ack = require_ack;

  const uint8_t *p = (const uint8_t *)&tx_buf.packet;
  for(int i = 0; i < tx_buf.packet.hdr.length + 1 /* len */ + 1 /* register */; i++) {
    RFM69_write_blocking(RFM69_REG_FIFO, p[i]);
  }

  // transmit
  RFM69_write_blocking(RFM69_REG_OPMODE, 0b011 << 2);

  // wait mode ready
  while(!(RFM69_read_blocking(0x27) & (1 << 7)));

  // waint until PacketSent
  while(!(RFM69_read_blocking(0x28) & (1 << 3)));
}

void RFM69_init(uint8_t node_id) {
  is_gateway = node_id == 0;
  tx_buf.packet.hdr.src = node_id;

  if(!is_gateway) {
    TX_LED_PORT = GPIOC;
    TX_LED_PIN = 6;
  }

  gpio_output(TX_LED_PORT, TX_LED_PIN, 0);
  gpio_output(RX_LED_PORT, RX_LED_PIN, 0);

  spi_init();
  tx_led_timer = timer_create();
  rx_led_timer = timer_create();

  const uint32_t bitrate = 4800;
  const uint32_t F_dev = 50000;

  const uint32_t F_osc = 32000000;
  const uint32_t F_step = F_osc / 524288;  // 524288 = 2**19
  const uint32_t networkID = 0xABCDEF;

  while(!spi_finished);

  // 868Mhz
  // Frf / (Fxosc / 2**19)
  // 868000000 / (32000000 / 2**19)
  RFM69_write_blocking(RFM69_REG_FRFMSB, 0xD9);
  RFM69_write_blocking(RFM69_REG_FRFMID, 0x00);
  RFM69_write_blocking(RFM69_REG_FRFLSB, 0x00);

  RFM69_write16(RFM69_REG_BITRATEMSB, F_osc / bitrate);
  RFM69_write16(RFM69_REG_FDEVMSB, F_dev / F_step);
  RFM69_write_blocking(RFM69_REG_RXBW, 0x42);

  RFM69_write_blocking(RFM69_REG_DIOMAPPING2, 0x07);

  // two sync bytes
  RFM69_write_blocking(RFM69_REG_SYNCCONFIG, 0x88);
  RFM69_write16(RFM69_REG_SYNCVALUE1, networkID & 0xFFFFU);

  // variable packet, and filter our address
  RFM69_write_blocking(RFM69_REG_PACKETCONFIG1, 0b10010010);
  RFM69_write_blocking(RFM69_REG_PAYLOADLENGTH, 1);
  RFM69_write_blocking(RFM69_REG_PAYLOADLENGTH, 66);
  RFM69_write_blocking(RFM69_REG_NODEADRS, node_id);

  // start sending once fifo has at least one byte
  RFM69_write_blocking(RFM69_REG_FIFOTHRESH, 0x80);

  // enable IRQ
  // Rx - PayloadReady
  // Tx - TxReady
  RFM69_write_blocking(RFM69_REG_DIOMAPPING1, 0b01 << 6);

  // sleep
  RFM69_write_blocking(RFM69_REG_OPMODE, 0);

  // enable IRQ pin as an input
  gpio_input(GPIOB, irq_pin);
}
