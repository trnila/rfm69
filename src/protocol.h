#pragma once
#include <stdint.h>

/*
  1: 0-29
  2: 30-59
  3: 60-89

room, open, vbat, fw
-> room, open
*/

struct SensorState {
  uint8_t open;
  uint16_t voltage;
  uint8_t firmware;
  uint8_t RSSI;
  uint8_t counter;
  uint8_t cnt_ack;
} __attribute__((packed));

struct SensorStateAck {
  uint8_t open;
  uint8_t fw;
  uint8_t ack;
} __attribute__((packed));
