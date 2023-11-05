#include <stdio.h>
#include <string.h>

#include "RFM69.h"
#include "adc.h"
#include "gpio.h"
#include "sensors.h"
#include "stm32g031xx.h"
#include "uart.h"

static const uint8_t node_id = 1;
extern RFM69_Packet *tx_packet;

void send_sensor(uint8_t id, uint32_t value) {
  uint8_t *payload = RFM69_get_tx_payload();
  if(!payload) {
    return;
  }

  payload[0] = RFM69_CMD_MEASUREMENT;
  payload[1] = id;
  payload[2] = value;
  payload[3] = value >> 8;
  payload[4] = value >> 16;
  payload[5] = value >> 24;

  RFM69_send_packet(0, true, 1 + 1 + sizeof(uint32_t));
}

void app_main() {
  RFM69_init(node_id);

  // for(;;) {
  //   adc_read();
  // }

  uint32_t counter = 0;
  for(;;) {
    if(RFM69_read_packet()) {
      RFM69_rxbuf_return();
    }

    if(counter % 100000 == 0) {
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
    }

    counter++;
  }
}
