// encoders.h
#pragma once
#include <stdint.h>

void encoder_init(void);
uint32_t encoder_get_ticks(void);
void encoder_reset_ticks(void);