#include <stdio.h>
#include <string.h>

#include "RFM69.h"
#include "adc.h"
#include "gpio.h"
#include "sensors.h"
#include "stm32g031xx.h"
#include "uart.h"

static const uint8_t node_id = 1;
static RFM69_Packet tx_packet;

void send_sensor(uint8_t id, uint32_t value) {
  RFM69_set_mode(RFM69_MODE_STANDBY);

  // send fifo
  uint8_t payload_len = 1 + 1 + sizeof(uint32_t);
  tx_packet.hdr.length = sizeof(tx_packet.hdr) - 1 + payload_len;
  tx_packet.hdr.dst = 0;
  tx_packet.hdr.src = node_id;
  tx_packet.hdr.seq = 0;

  tx_packet.payload[0] = RFM69_CMD_MEASUREMENT;
  tx_packet.payload[1] = id;
  tx_packet.payload[2] = value;
  tx_packet.payload[3] = value >> 8;
  tx_packet.payload[4] = value >> 16;
  tx_packet.payload[5] = value >> 24;

  uint8_t *ptr = (uint8_t *)&tx_packet;
  for(size_t i = 0; i < tx_packet.hdr.length + 1; i++) {
    RFM69_write(RFM69_REG_FIFO, ptr[i]);
  }

  RFM69_set_mode(RFM69_MODE_TX);

  // packet sent
  while((RFM69_read(RFM69_REG_IRQFLAGS2) & (1 << 3)) == 0)
    ;

  gpio_set(GPIOC, 6, 0);
  for(uint32_t i = 0; i < 100000; i++)
    ;  // delay
  gpio_set(GPIOC, 6, 1);
}

void app_main() {
  RFM69_init(node_id);

  // for(;;) {
  //   adc_read();
  // }

  uint32_t counter = 0;
  for(;;) {
    adc_measurements_t m;
    adc_read(&m);

    send_sensor(SENSOR_BATTERY_CURRENT, m.bat_I_mV);
    send_sensor(SENSOR_BATTERY_VOLTAGE, m.bat_V_mV);
    send_sensor(SENSOR_SOLAR_VOLTAGE, m.solar_V_mV);
    send_sensor(SENSOR_VBAT, m.vbat_mV);
    send_sensor(SENSOR_VREF, m.vref_mV);

    /*
    send_sensor(0, counter);
    send_sensor(1, counter * 10);
    send_sensor(2, counter * 100);
    send_sensor(3, counter % 40);
    send_sensor(4, counter % 100);
    send_sensor(5, counter % 10 < 5);
    send_sensor(6, 100 - counter % 100);
    */

    counter++;
    for(uint32_t i = 0; i < 1000000; i++)
      ;  // delay
  }
}
