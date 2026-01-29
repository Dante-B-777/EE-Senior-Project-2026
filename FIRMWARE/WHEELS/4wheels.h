#ifndef INC_WHEELS4_H_
#define INC_WHEELS4_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

typedef enum {
  W_FL = 0,
  W_FR = 1,
  W_RL = 2,
  W_RR = 3
} WheelIndex;

void Wheels4_Init(UART_HandleTypeDef *huart);

void Wheels4_ConfigAll(
    TIM_HandleTypeDef *htim2,
    GPIO_TypeDef *dirFL_port, uint16_t dirFL_pin,
    GPIO_TypeDef *dirFR_port, uint16_t dirFR_pin,
    GPIO_TypeDef *dirRL_port, uint16_t dirRL_pin,
    GPIO_TypeDef *dirRR_port, uint16_t dirRR_pin);

void Wheels4_OnRxByte(uint8_t b);
void Wheels4_Task(uint32_t now_ms);

void Wheel_SetSignedSpeed(WheelIndex idx, int16_t cmd);
void Wheels4_SetAll(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
void Wheels4_Stop(void);

#endif
