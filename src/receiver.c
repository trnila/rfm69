#include <stdio.h>
#include <stdlib.h>

#include "RFM69.h"
#include "gpio.h"
#include "rgb.h"
#include "stm32g031xx.h"
#include "timer.h"
#include "uart.h"

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

const uint8_t map[23] = {
    0, 0, 0, 0, 1,
};

typedef enum {
  WINDOW_OPEN,
  WINDOW_CLOSED,
  WINDOW_TIMEDOUT,
} WindowState;

typedef struct {
  WindowState state;
  uint32_t last_update;
} Room;

Room room_last_updated[sizeof(map)];

void app_main() {
  RFM69_init(0);
  rgb_init();
  rgb_set_brightness(10);

  for(size_t i = 0; i < sizeof(map); i++) {
    rgb_set(i, 255, 255, 0);
  }

  for(;;) {
    RFM69_Packet *packet = RFM69_read_packet();
    if(packet) {
      if(packet->payload[1] == 9) {
        for(size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
          if(map[i] == packet->hdr.src) {
            bool closed = packet->payload[2];
            room_last_updated[i].state = closed ? WINDOW_CLOSED : WINDOW_OPEN;
            room_last_updated[i].last_update = tick_ms;
            rgb_set(i, 255 * !closed, 0, 0);
            break;
          }
        }
      }

      char buf[32];
      snprintf(buf, sizeof(buf), "%d %d %d\n", packet->hdr.flags.seq, packet->payload[1],
               packet->payload[2] | (packet->payload[3] << 8) | (packet->payload[4] << 16) | packet->payload[5] << 24);

      RFM69_rxbuf_return();
      uart_send(buf);
    }

    for(size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
      if(tick_ms - room_last_updated[i].last_update > 5000 && room_last_updated[i].state != WINDOW_TIMEDOUT) {
        rgb_set(i, 255, 255, 0);
        room_last_updated[i].state = WINDOW_TIMEDOUT;
      }
    }

    rgb_update();
  }
}
