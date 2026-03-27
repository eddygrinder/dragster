// motors_control.h
#pragma once
#include <stdint.h>
#include "driver/ledc.h" // ledc_channel_t

// ----- Motores -----
#define AIN1 17 // GPIO17 — LPWM esquerdo
#define AIN2 16 // GPIO16 — RPWM esquerdo
#define PWM1 15 
#define BIN1 40 // GPIO40 — LPWM direito
#define BIN2 41 // GPIO41 — RPWM direito
#define PWM2 42 

#define MOTOR_PWM_FREQ 20000 ///< Frequência PWM (Hz)
#define MOTOR_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER LEDC_TIMER_0
#define MOTOR_PWM_RES LEDC_TIMER_10_BIT      ///< Resolução PWM (10-bit)
#define MAX_DUTY_CYCLE 1023                  ///< Ciclo máximo para 10-bit
#define MOTOR_PWM_CHANNEL_ESQ LEDC_CHANNEL_0 ///< Canal PWM do motor esquerdo
#define MOTOR_PWM_CHANNEL_DTA LEDC_CHANNEL_1 ///< Canal PWM do motor direito
#define MOTOR_PWM_CHANNEL_ESQ_BRAKE LEDC_CHANNEL_2
#define MOTOR_PWM_CHANNEL_DTA_BRAKE LEDC_CHANNEL_3

/**
 * @brief Ajusta os motores esquerdo e direito com base na posição da linha.
 *
 * @param line_position Posição calculada da linha (-1.0 à esquerda, +1.0 à direita)
 */
void motors_init(void);   // nova função
void motorControl(float line_position);
//static void motor_set(int pwm, int gpio_a, int gpio_b, ledc_channel_t channel);
void motors_set(int left_pwm, int right_pwm);
void motors_stop_fast(void);
void motors_coast(void);
void motors_brake_set(int pwm);