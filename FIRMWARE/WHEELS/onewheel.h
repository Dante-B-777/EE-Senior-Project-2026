
#include "stm32l4xx_hal.h"
#include <stdint.h>
#ifndef INC_WHEELS_H_
#define INC_WHEELS_H_

/* ================= MOTOR PIN DEFINITIONS ================= */
/* PWM: PA0 -> TIM2_CH1 (AF1) */
#define MOTOR_PWM_PORT         GPIOA
#define MOTOR_PWM_PIN          GPIO_PIN_0

/* DIR: PB0 -> GPIO output */
#define MOTOR_DIR_GPIO_PORT    GPIOB
#define MOTOR_DIR_PIN          GPIO_PIN_0

/* ================= ENCODER PIN DEFINITIONS ================= */
/* Encoder A: PA6 -> TIM3_CH1 (AF2) */
/* Encoder B: PA7 -> TIM3_CH2 (AF2) */
#define ENC_PORT               GPIOA
#define ENC_A_PIN              GPIO_PIN_6
#define ENC_B_PIN              GPIO_PIN_7

/* ================= LPUART1 (ST-LINK VCP) ================= */
/* ST-LINK Virtual COM typically uses:
   PG7 = LPUART1_RX (AF8)
   PG8 = LPUART1_TX (AF8)
*/
#define STLK_RX_Pin            GPIO_PIN_7
#define STLK_RX_GPIO_Port      GPIOG
#define STLK_TX_Pin            GPIO_PIN_8
#define STLK_TX_GPIO_Port      GPIOG

/* ================= FLOAT ENCODER CONSTANTS =================
   From your motor sheet:
   - Encoder PPR at motor shaft = 28
   - Gear ratio = 19.2:1
   We use TIM3 in encoder mode with x4 decoding:
     CPR_motor = 4 * PPR = 112 counts/rev
     CPR_wheel = CPR_motor * gear_ratio ≈ 2150.4 counts/rev
*/
#define ENC_PPR_MOTOR_SHAFT     (28.0f)
#define GEAR_RATIO              (19.2f)
#define CPR_MOTOR_SHAFT         112      /* 112 */
#define CPR_GEARBOX_OUTPUT      (CPR_MOTOR_SHAFT * GEAR_RATIO)     /* ~2150.4 */

/* ---------------- INT-only encoder constants ----------------
   Motor encoder: 28 PPR, quadrature x4 => 112 counts/rev on motor shaft.
   Gear ratio: 19.2:1 = 192/10 (exact rational), so wheel rev is computed EXACTLY with ints:
      wheel_revs = motor_revs / 19.2
   We print:
     - rev as X.YYY  (milli-rev)
     - angle as A.BB (centi-deg)
*/
#define CPR_MOTOR_SHAFT_I   505       /* 4 * 28 */
#define GR_NUM             192        /* 19.2 = 192/10 */
#define GR_DEN              10



/* ---------------- Prototypes ---------------- */
void Motor_GPIO_Init(void);
void TIM2_PWM_Init_20kHz(void);
void Motor_SetSignedSpeed(int16_t cmd);

void Encoder_GPIO_Init(void);
void TIM3_Encoder_Init(void);
int32_t Encoder_GetCount32(void);

void LPUART1_Init_115200_MSI4MHz(void);
void uart_write(const char *s);
void uart_printf(const char *fmt, ...);

int32_t iabs32(int32_t x);
void Print_Telemetry_IntOnly(int32_t cnt);




















#endif /* INC_WHEELS_H_ */
