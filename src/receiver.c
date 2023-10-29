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

  // rx
  RFM69_set_mode(RFM69_MODE_RX);

  for(;;) {
    size_t len;
    uint8_t *data = RFM69_readmsg(&len);
    if(data) {
      data[len] = 0;
      uart_send((char *)data);
    }
  }
}
