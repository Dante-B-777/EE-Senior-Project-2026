#ifndef INC_ODOM4_H_
#define INC_ODOM4_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
  // encoder timers (configured in Encoder Interface mode)
  TIM_HandleTypeDef *enc_fl;
  TIM_HandleTypeDef *enc_fr;
  TIM_HandleTypeDef *enc_rl;
  TIM_HandleTypeDef *enc_rr;

  // previous raw counts
  int32_t prev_fl;
  int32_t prev_fr;
  int32_t prev_rl;
  int32_t prev_rr;

  // pose
  float x_m;
  float y_m;
  float theta_rad;

  // velocities
  float v_mps;   // linear velocity (m/s)
  float w_rps;   // angular velocity (rad/s)

  // params
  float wheel_radius_m;
  float track_width_m;
  float counts_per_rev;   // timer counts per wheel revolution (after quadrature decoding)

  // encoder sign multipliers (+1 or -1) to fix wiring/convention
  int8_t sign_fl;
  int8_t sign_fr;
  int8_t sign_rl;
  int8_t sign_rr;

  uint32_t last_ms;
  uint8_t initialized;
} Odom4_t;

void Odom4_Init(Odom4_t *o,
                TIM_HandleTypeDef *enc_fl,
                TIM_HandleTypeDef *enc_fr,
                TIM_HandleTypeDef *enc_rl,
                TIM_HandleTypeDef *enc_rr,
                float wheel_radius_m,
                float track_width_m,
                float counts_per_rev);

void Odom4_Reset(Odom4_t *o);              // sets (x,y,theta)=0 and re-zeros deltas
void Odom4_Update(Odom4_t *o, uint32_t now_ms);

// Optional helper: format "O x y th v w\n" into buf, returns bytes written (>=0) or -1.
int Odom4_FormatTelemetry(const Odom4_t *o, char *buf, size_t buf_sz);

#endif
