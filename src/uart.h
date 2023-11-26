#pragma once

void uart_init(uint32_t clk_in, uint32_t bauds);
void uart_send(const char* str);
