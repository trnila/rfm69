#include "RFM69.h"
#include "gpio.h"
#include "rgb.h"
#include "stm32g031xx.h"
#include "uart.h"

char data[66];

void app_main() {
  RFM69_init();
  rgb_init();
  rgb_set_brightness(10);

  // rx
  RFM69_set_mode(RFM69_MODE_RX);

  {
    for(int color = 0; color < 4; color++) {
      for(int led = 0; led < 23; led++) {
        rgb_set(led, (color == 0) * 255, (color == 1) * 255,
                (color == 2) * 255);
        rgb_update();

        for(uint32_t i = 0; i < 100000; i++)
          ;  // delay
      }
    }
  }

  for(;;) {
    uint8_t val = RFM69_read(RFM69_REG_VERSION);
    if(val != 0x24) {
      gpio_set(GPIOC, 6, 1);
      for(;;)
        ;
    }

    uint8_t flags2 = RFM69_read(RFM69_REG_IRQFLAGS2);
    if(flags2 & (1 << 2)) {
      uint8_t len = RFM69_read(RFM69_REG_FIFO);
      if(len >= sizeof(data)) {
        len = sizeof(data) - 1;
      }
      data[len] = 0;

      for(uint8_t i = 0; i < len; i++) {
        data[i] = RFM69_read(RFM69_REG_FIFO);
      }

      uart_send(data);
    }
  }
}
