#pragma once
#include <stddef.h>
#include <stdint.h>

void rgb_init(void);
void rgb_set_brightness(uint8_t br);
void rgb_set(size_t pos, uint8_t r, uint8_t g, uint8_t b);
void rgb_clear();
void rgb_update();
