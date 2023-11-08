#pragma once

#include <stdint.h>

extern uint32_t tick_ms;

typedef struct timer timer;

typedef void (*timer_callback)(timer* t);

void timer_init();

timer* timer_create();
void timer_oneshot(timer* t, uint32_t delay_ms, timer_callback cb);
void timer_stop(timer* t);
