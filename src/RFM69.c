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

typedef enum {
  RFM69_STATE_SLEEP = 0,
  RFM69_STATE_STANDBY = 1,
  RFM69_STATE_TX = 3,
  RFM69_STATE_RX = 4,
  RFM69_STATE_NONE,

  RFM69_STATE_RX_FIFO_LEN,
  RFM69_STATE_RX_FIFO_DATA,
  RFM69_STATE_RX_SET_IRQ,

  RFM69_STATE_TX_FIFO,
  RFM69_STATE_TX_SET_IRQ,
} RFM69_State;

typedef struct __attribute__((packed)) {
  uint8_t reg;
  RFM69_Packet packet;
} DMABuffer;

static GPIO_TypeDef *TX_LED_PORT = GPIOB;
static unsigned int TX_LED_PIN = 1;
static GPIO_TypeDef *RX_LED_PORT = GPIOA;
static unsigned int RX_LED_PIN = 10;
static const uint32_t irq_pin = 6;

static timer *tx_ack_timeout_timer;
static timer *rx_led_timer;
static timer *tx_led_timer;

static DMABuffer rx_buf;
static volatile bool rx_buf_full = false;
static uint8_t rx_fifo_len;
static bool payload_ready = 0;

static DMABuffer tx_buf;
static volatile bool packet_sent = 0;
static volatile bool tx_buf_full = false;
static volatile bool tx_ack_timedout;
static uint8_t tx_ack_retries = 0;

static RFM69_State state = RFM69_STATE_NONE;
static bool is_gateway;

static volatile bool spi_finished = true;
static uint8_t tx_spi_buf[2];
static uint8_t rx_spi_buf[2];

static RFM69_Packet *tx_packet = &tx_buf.packet;

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

  process_state_machine();
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
  while(spi_finished == false)
    ;
  __DMB();

  return rx[1];
}

void RFM69_write_blocking(uint8_t addr, uint8_t val) { RFM69_transmit_blocking(addr | RFM69_WRITE_MASK, val); }

void RFM69_write16(uint8_t addr, uint16_t val) {
  RFM69_write_blocking(addr, val >> 8);
  RFM69_write_blocking(addr + 1, val);
}

uint8_t RFM69_read_blocking(uint8_t addr) { return RFM69_transmit_blocking(addr, 0); }

void RFM69_set_mode(RFM69_State req_state) {
  assert(req_state < RFM69_STATE_NONE);

  state = req_state;

  tx_spi_buf[0] = RFM69_REG_OPMODE | RFM69_WRITE_MASK;
  tx_spi_buf[1] = req_state << 2;

  spi_transmit(tx_spi_buf, rx_spi_buf, 2);
}

static void RFM69_set_rx_irq() {
  tx_spi_buf[0] = RFM69_REG_DIOMAPPING1 | RFM69_WRITE_MASK;
  tx_spi_buf[1] = 0b01 << 6;
  spi_transmit(tx_spi_buf, rx_spi_buf, 2);
}

void tx_no_ack_timeout(timer *t) {
  tx_ack_timedout = true;
  process_state_machine();
}

void rx_led_off(timer *t) { gpio_output(RX_LED_PORT, RX_LED_PIN, 0); }

void tx_led_off(timer *t) { gpio_output(TX_LED_PORT, TX_LED_PIN, 0); }

void process_state_machine() {
  __disable_irq();
  switch(state) {
    case RFM69_STATE_RX:
      if(payload_ready || tx_ack_timedout) {
        gpio_output(RX_LED_PORT, RX_LED_PIN, 1);
        timer_oneshot(rx_led_timer, 50, rx_led_off);
        RFM69_set_mode(RFM69_STATE_SLEEP);
      }
      break;

    case RFM69_STATE_SLEEP:
      if(spi_finished && !rx_buf_full && payload_ready) {
        state = RFM69_STATE_RX_FIFO_LEN;
        tx_spi_buf[0] = RFM69_REG_FIFO;
        spi_transmit(tx_spi_buf, rx_spi_buf, 2);
      } else if(spi_finished && !rx_buf_full && !payload_ready && is_gateway) {
        state = RFM69_STATE_RX_SET_IRQ;
        RFM69_set_rx_irq();
      } else if(spi_finished && tx_buf_full && (tx_ack_retries == 0 || tx_ack_timedout)) {
        ++tx_ack_retries;
        packet_sent = false;
        tx_ack_timedout = false;
        state = RFM69_STATE_TX_FIFO;
        tx_buf.reg = RFM69_REG_FIFO | RFM69_WRITE_MASK;
        spi_transmit((uint8_t *)&tx_buf, NULL, tx_buf.packet.hdr.length + 1 /* len */ + 1 /* register */);
      }
      break;

    case RFM69_STATE_RX_FIFO_LEN:
      if(spi_finished) {
        state = RFM69_STATE_RX_FIFO_DATA;
        rx_fifo_len = rx_spi_buf[1];
        assert(rx_fifo_len <= sizeof(rx_buf.packet));
        // NULL reads from register FIFO
        spi_transmit(NULL, (uint8_t *)&rx_buf.packet, rx_fifo_len + 1);
      }
      break;

    case RFM69_STATE_RX_FIFO_DATA:
      if(spi_finished) {
        payload_ready = false;
        if(rx_buf.packet.hdr.flags.req_ack) {
          uint8_t payload_len = 2;
          tx_buf.packet.payload[0] = RFM69_CMD_ACK;
          tx_buf.packet.payload[1] = rx_buf.packet.hdr.flags.seq;

          tx_buf.packet.hdr.dst = rx_buf.packet.hdr.src;
          tx_buf.packet.hdr.flags.req_ack = false;
          tx_buf.packet.hdr.length = payload_len + sizeof(tx_packet->hdr) - 1;
          tx_buf.packet.hdr.flags.seq++;

          // TODO: use different txbuf!
          tx_buf_full = true;
          tx_ack_retries = 0;
        } else if(rx_buf.packet.payload[0] == RFM69_CMD_ACK &&
                  rx_buf.packet.payload[1] == tx_buf.packet.hdr.flags.seq) {
          timer_stop(tx_ack_timeout_timer);
          tx_buf_full = false;
        }

        RFM69_set_mode(RFM69_STATE_SLEEP);

        rx_buf_full = true;
        rx_buf.packet.hdr.length = rx_fifo_len;
      }
      break;

    case RFM69_STATE_RX_SET_IRQ:
      if(spi_finished) {
        RFM69_set_mode(RFM69_STATE_RX);
      }
      break;

    case RFM69_STATE_TX_FIFO:
      if(spi_finished) {
        state = RFM69_STATE_TX_SET_IRQ;
        // packet sent
        tx_spi_buf[0] = RFM69_REG_DIOMAPPING1 | RFM69_WRITE_MASK;
        tx_spi_buf[1] = 0;
        spi_transmit(tx_spi_buf, rx_spi_buf, 2);
      }
      break;

    case RFM69_STATE_TX_SET_IRQ:
      if(spi_finished) {
        RFM69_set_mode(RFM69_STATE_TX);
      }
      break;

    case RFM69_STATE_TX:
      if(packet_sent && spi_finished) {
        gpio_output(TX_LED_PORT, TX_LED_PIN, 1);
        timer_oneshot(tx_led_timer, 50, tx_led_off);
        if(!tx_buf.packet.hdr.flags.req_ack && !is_gateway) {
          tx_buf_full = false;
          RFM69_set_mode(RFM69_STATE_SLEEP);
        } else {
          if(tx_buf.packet.hdr.flags.req_ack) {
            timer_oneshot(tx_ack_timeout_timer, 1000, tx_no_ack_timeout);
          } else {
            tx_buf_full = false;
          }
          state = RFM69_STATE_RX_SET_IRQ;
          RFM69_set_rx_irq();
        }
      }
      break;

    case RFM69_STATE_NONE:
    case RFM69_STATE_STANDBY:
      break;
  }
  __enable_irq();
}

void EXTI4_15_IRQHandler() {
  EXTI->RPR1 = 1 << irq_pin;

  if(state == RFM69_STATE_RX) {
    payload_ready = true;
  } else if(state == RFM69_STATE_TX) {
    packet_sent = true;
  }

  process_state_machine();
}

void RFM69_rxbuf_return() {
  rx_buf_full = false;
  process_state_machine();
}

RFM69_Packet *RFM69_read_packet() {
  if(!rx_buf_full) {
    return NULL;
  }

  __DMB();
  return &rx_buf.packet;
}

uint8_t *RFM69_get_tx_payload() {
  uint8_t *payload = NULL;
  __disable_irq();
  if(!tx_buf_full) {
    payload = tx_buf.packet.payload;
  }
  __enable_irq();
  return payload;
}

void RFM69_send_packet(uint8_t dst, bool require_ack, uint8_t payload_len) {
  tx_buf.packet.hdr.length = payload_len + sizeof(tx_packet->hdr) - 1;
  tx_buf.packet.hdr.dst = dst;
  tx_buf.packet.hdr.flags.seq++;
  tx_buf.packet.hdr.flags.req_ack = require_ack;
  tx_ack_retries = 0;
  tx_buf_full = true;
  process_state_machine();
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
  tx_ack_timeout_timer = timer_create();
  tx_led_timer = timer_create();
  rx_led_timer = timer_create();

  const uint32_t bitrate = 4800;
  const uint32_t F_dev = 50000;

  const uint32_t F_osc = 32000000;
  const uint32_t F_step = F_osc / 524288;  // 524288 = 2**19
  const uint32_t networkID = 0xABCDEF;

  RFM69_set_mode(RFM69_STATE_SLEEP);
  while(!spi_finished)
    ;

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

  // enable IRQ pin
  gpio_input(GPIOB, irq_pin);
  // source is portB
  EXTI->EXTICR[irq_pin / 4] = 0x01 << ((irq_pin & 3) << 3);
  // Rising edge
  EXTI->RTSR1 |= 1U << irq_pin;
  // unask irq - yeah, we are unasking according to docs ^^
  EXTI->IMR1 |= 1U << irq_pin;
  NVIC_EnableIRQ(EXTI4_15_IRQn);

  process_state_machine();
}
