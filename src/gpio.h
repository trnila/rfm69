#include "stm32g031xx.h"

inline void gpio_mode(GPIO_TypeDef* base, int pin, int mode) {
  base->MODER &= ~(GPIO_MODER_MODE0 << (pin << 1));
  base->MODER |= (mode & 0b11) << (pin << 1);
}

inline void gpio_alt(GPIO_TypeDef* base, int pin, int alt) {
  // set alt
  gpio_mode(base, pin, 0b10);

  const uint32_t shift = (pin >= 8 ? pin - 8 : pin);
  uint32_t val = (alt & 0xF) << (shift << 2);

  if(pin >= 8) {
    base->AFR[1] |= val;
  } else {
    base->AFR[0] |= val;
  }
}

inline void gpio_set(GPIO_TypeDef* base, int pin, int state) {
  if(state) {
    base->BSRR = 1 << pin;
  } else {
    base->BRR = 1 << pin;
  }
}

inline unsigned int gpio_read(GPIO_TypeDef* base, int pin) { return (base->IDR & (1U << pin)) > 0; }

inline void gpio_output(GPIO_TypeDef* base, int pin, int state) {
  gpio_set(base, pin, state);
  gpio_mode(base, pin, 0b01);
}

inline void gpio_input(GPIO_TypeDef* base, int pin) { gpio_mode(base, pin, 0b00); }

inline void gpio_speed(GPIO_TypeDef* base, unsigned int pin, uint8_t speed) {
  base->OSPEEDR |= (speed & 0b11) << (pin << 1);
}

inline void gpio_pullup(GPIO_TypeDef* base, unsigned int pin) {
  const unsigned int shift = pin << 1;
  base->PUPDR = (base->PUPDR & ~(0b11 << shift)) | (0b01 << shift);
}

inline void gpio_pulldown(GPIO_TypeDef* base, unsigned int pin) {
  const unsigned int shift = pin << 1;
  base->PUPDR = (base->PUPDR & ~(0b11 << shift)) | (0b10 << shift);
}
