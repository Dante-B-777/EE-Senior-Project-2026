#ifndef INC_WHEELS4_H_
#define INC_WHEELS4_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    W_FL = 0,   // Front Left
    W_FR = 1,   // Front Right
    W_RL = 2,   // Rear Left
    W_RR = 3    // Rear Right
} WheelIndex;

typedef struct {
    TIM_TypeDef *pwm_tim;
    uint8_t      pwm_ch;      // 1..4 (maps to CCR1..CCR4)

    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    bool          dir_fwd_is_low;  // match your driver wiring

    TIM_TypeDef *enc_tim;

    // extended count state
    uint16_t enc_prev;
    int32_t  enc_acc;
} Wheel_t;

extern Wheel_t g_wheels[4];

/* Init */
void Wheels_GPIO_Init_All(void);
void Wheels_PWM_TIM2_Init_20kHz_All4(void);
void Wheels_Enc_Init_All(void);

/* Per-wheel control */
void Wheel_SetSignedSpeed(WheelIndex w, int16_t cmd); // -1000..+1000
int32_t Wheel_GetCount32(WheelIndex w);

/* High-level robot motions */
void Drive_Stop(void);
void Drive_Forward(int16_t cmd);
void Drive_Reverse(int16_t cmd);
void Drive_TurnLeft(int16_t cmd);
void Drive_TurnRight(int16_t cmd);

#endif

