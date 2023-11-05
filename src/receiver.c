#include <stdio.h>
#include <stdlib.h>

#include "RFM69.h"
#include "gpio.h"
#include "rgb.h"
#include "stm32g031xx.h"
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

void app_main() {
  RFM69_init(0);
  rgb_init();
  rgb_set_brightness(10);

  for(;;) {
    RFM69_Packet *packet = RFM69_read_packet();
    if(packet) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%d %d %d\n", packet->hdr.flags.seq, packet->payload[1],
               packet->payload[2] | (packet->payload[3] << 8) | (packet->payload[4] << 16) | packet->payload[5] << 24);

      RFM69_rxbuf_return();
      uart_send(buf);
    }
  }
}
