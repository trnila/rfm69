#include "timer.h"

#include <stdbool.h>
#include <stddef.h>

#include "stm32g031xx.h"

uint32_t tick_ms;

typedef struct timer {
  bool active;
  bool created;
  uint32_t scheduled_at;
  uint32_t period;
  timer_callback callback;
} timer;

static timer timers[4];

void SysTick_Handler() {
  // reload counter
  SysTick->VAL = 0;
  ++tick_ms;

  for(size_t i = 0; i < sizeof(timers) / sizeof(timers[0]); i++) {
    timer* t = &timers[i];
    if(t->active && t->scheduled_at < tick_ms) {
      t->callback(t);
      t->active = false;
    }
  }
}

void timer_init() {
  NVIC_EnableIRQ(SysTick_IRQn);
  // SYSCLK (HSI 16M) / AHB prescaler (1) / 8 (systick source 1)
  uint32_t systick_freq = 16000000U / 1U / 8U;
  SysTick->LOAD = systick_freq * 1U / 1000U;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

timer* timer_create() {
  for(size_t i = 0; i < sizeof(timers) / sizeof(timers[0]); i++) {
    timer* t = &timers[i];
    if(!t->created) {
      t->created = true;
      return t;
    }
  }

  asm("bkpt #1");
  return NULL;
}

void timer_oneshot(timer* t, uint32_t delay_ms, timer_callback cb) {
  __disable_irq();
  t->scheduled_at = tick_ms + delay_ms;
  t->callback = cb;
  t->active = true;
  __enable_irq();
}

void timer_stop(timer* t) {
  __disable_irq();
  t->active = false;
  __enable_irq();
}
