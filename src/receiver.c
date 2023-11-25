#include <stdio.h>
#include <stdlib.h>

#include "RFM69.h"
#include "gpio.h"
#include "rgb.h"
#include "sensors.h"
#include "stm32g031xx.h"
#include "timer.h"
#include "uart.h"

#define MAX_ROOMS 23

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

      for(uint32_t i = 0; i < 100000; i++)
        ;  // delay
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

static void handle_measurement(uint8_t src, uint8_t seq, uint8_t sensor_id, uint32_t value) {
  if(sensor_id == SENSOR_WINDOW) {
    for(size_t i = 0; i < config->node_count; i++) {
      if(config->node_to_led[i] == src) {
        bool open = value != 1;
        room_last_updated[i].state = open ? WINDOW_OPEN : WINDOW_CLOSED;
        room_last_updated[i].last_update = tick_ms;
        rgb_set(i, 255 * open, 0, 0);
        break;
      }
    }
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "%d %d %ld\n", seq, sensor_id, value);
  uart_send(buf);
}

void main() {
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  timer_init();
  uart_init();

  RFM69_init(0);
  assert(config->node_count <= MAX_ROOMS);
  rgb_init(config->node_count);
  rgb_set_brightness(10);

  for(size_t i = 0; i < config->node_count; i++) {
    rgb_set(i, 255, 255, 0);
  }

  for(;;) {
    RFM69_Packet *packet = RFM69_read_packet();
    if(packet) {
      size_t offset = 0;
      while(offset < (packet->hdr.length - sizeof(packet->hdr) + 1)) {
        const uint8_t *data = &packet->payload[offset];
        if(data[0] == RFM69_CMD_MEASUREMENT) {
          uint32_t value = data[2] | (data[3] << 8) | (data[4] << 16) | data[5] << 24;
          handle_measurement(packet->hdr.src, packet->hdr.flags.seq, data[1], value);
          offset += 2 + 4;
        } else {
          assert(0);
        }
      }
      RFM69_rxbuf_return();
    }

    for(size_t i = 0; i < config->node_count; i++) {
      if(tick_ms - room_last_updated[i].last_update > 5000 && room_last_updated[i].state != WINDOW_TIMEDOUT) {
        rgb_set(i, 255, 255, 0);
        room_last_updated[i].state = WINDOW_TIMEDOUT;
      }
    }

    rgb_update();
  }
}
