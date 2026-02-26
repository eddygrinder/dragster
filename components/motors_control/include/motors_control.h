// motors_control.h
#pragma once
#include <stdint.h>
#include "driver/ledc.h" // ledc_channel_t

/**
 * @brief Ajusta os motores esquerdo e direito com base na posição da linha.
 *
 * @param line_position Posição calculada da linha (-1.0 à esquerda, +1.0 à direita)
 */
void motors_init(void);   // nova função
void motorControl(float line_position);
//static void motor_set(int pwm, int gpio_a, int gpio_b, ledc_channel_t channel);
void motors_set(int left_pwm, int right_pwm);
void motors_stop(void);

