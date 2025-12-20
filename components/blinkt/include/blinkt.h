// blinkt.h
#pragma once
#include <stdint.h>

void blinkt_init(void);
void blinkt_set_all(uint8_t r, uint8_t g, uint8_t b, float brightness);
void blinkt_white(void);
void blinkt_clear(void);
