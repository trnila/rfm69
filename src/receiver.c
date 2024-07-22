#include <stdio.h>
#include <stdlib.h>

#include "RFM69.h"
#include "config.h"
#include "gpio.h"
#include "protocol.h"
#include "rgb.h"
#include "stm32g031xx.h"
#include "timer.h"
#include "uart.h"

#define MAX_ROOMS 23
#define TIMEOUT_MS 20000

typedef struct {
  uint8_t node_count;
  uint8_t node_to_led[MAX_ROOMS];
} Config;

const Config *config = (const Config *)0x0800F800;

void rgb_test() {
  for(int color = 0; color < 4; color++) {
    for(int led = 0; led < 23; led++) {
      rgb_set(led, (color == 0) * 255, (color == 1) * 255, (color == 2) * 255);
      rgb_update();

      for(uint32_t i = 0; i < 100000; i++);  // delay
    }
  }
}

typedef enum {
  WINDOW_OPEN,
  WINDOW_CLOSED,
  WINDOW_TIMEDOUT,
} WindowState;

typedef struct {
  WindowState state;
  uint32_t last_update;
} Room;

Room room_last_updated[MAX_ROOMS];

/*
static void handle_measurement(uint8_t src, uint8_t seq, uint32_t value) {
  for(size_t i = 0; i < config->node_count; i++) {
    if(config->node_to_led[i] == src) {
      bool open = value != 1;
      room_last_updated[i].state = open ? WINDOW_OPEN : WINDOW_CLOSED;
      room_last_updated[i].last_update = tick_ms;
      rgb_set(i, 255 * open, 0, 0);
      break;
    }
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "%d %ld\n", seq, value);
  uart_send(buf);
}
*/
char buf[128];
void main() {
  const uint32_t SYSCLK = 16000000UL;
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  timer_init(SYSCLK);
  uart_init(SYSCLK, 115200U);

  RFM69_init(0);
  assert(config->node_count <= MAX_ROOMS);
  rgb_init(config->node_count);
  rgb_set_brightness(10);

  for(size_t i = 0; i < config->node_count; i++) {
    rgb_set(i, 255, 255, 0);
  }
  uart_send("boot\n");

  uint8_t ack = 0;
  for(;;) {
    RFM69_Packet *packet = RFM69_read_packet();
    if(packet) {
      snprintf(buf, sizeof(buf), "len=%x dst=%x src=%x ", packet->hdr.length, packet->hdr.dst, packet->hdr.src);
      uart_send(buf);

      for(int i = 0; i < packet->hdr.length - sizeof(packet->hdr) + 1; i++) {
        snprintf(buf, sizeof(buf), "%02x ", packet->payload[i]);
        uart_send(buf);
      }

      extern uint8_t RSSI;
      struct SensorState *state = (struct SensorState *)packet->payload;
      snprintf(buf, sizeof(buf), "open=%d vbat=%d firmware=%x RSSI=% 4d dbm SRSSI=% 4d dbm cnt=% 3d ack=% 3d\n", state->open, state->voltage, state->firmware, -RSSI / 2, -state->RSSI / 2, state->counter, state->cnt_ack);
      uart_send(buf);

      if(packet->hdr.src < MAX_ROOMS) {
        Room *room = &room_last_updated[packet->hdr.src];
        room->state = state->open ? WINDOW_OPEN : WINDOW_CLOSED;
        room->last_update = tick_ms;
        rgb_set(packet->hdr.src, 255 * state->open, 0, 0);
      }

      for(uint32_t i = 0; i < 100000; i++) {
        asm("nop");
      }
      struct SensorStateAck *payload = (struct SensorStateAck *)RFM69_get_tx_payload();
      payload->open = state->open;
      payload->fw = state->firmware + 1;
      payload->ack = ack++;
      RFM69_send_packet(packet->hdr.src, true, sizeof(*payload));
    }

    for(size_t i = 0; i < config->node_count; i++) {
      if(tick_ms - room_last_updated[i].last_update > KEEPALIVE_MS + KEEPALIVE_TOLERANCE_MS && room_last_updated[i].state != WINDOW_TIMEDOUT) {
        rgb_set(i, 255, 255, 0);
        room_last_updated[i].state = WINDOW_TIMEDOUT;
      }
    }

    rgb_update();
  }
}
