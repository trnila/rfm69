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
} __attribute__((packed));