/**
 * @file    odom4.c
 * @brief   Four-wheel skid-steer/differential-drive odometry estimation
 *
 * Outputs pose (x,y,theta) and velocities (v,w) from four encoder timers.
 * Left distance = average(FL, RL), Right distance = average(FR, RR).
 *
 * Serial telemetry helper produces:
 *   O x y th v w\n
 */

#include "odom4.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float wrap_pi(float a) {
  while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
  while (a < -(float)M_PI) a += 2.0f*(float)M_PI;
  return a;
}

/**
 * Robust delta for any timer width, using ARR for modulo wrap.
 * Assumes the counter runs 0..ARR and wraps.
 *
 * Works for both 16-bit and 32-bit timers as long as ARR matches the configured period.
 * Update rate should be fast enough that delta never exceeds +/- ARR/2 counts per update.
 */
static inline int32_t delta_mod(TIM_HandleTypeDef *htim, int32_t *prev)
{
  uint32_t cur = __HAL_TIM_GET_COUNTER(htim);
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  uint32_t mod = arr + 1u;

  int32_t d = (int32_t)(cur - (uint32_t)(*prev));

  // wrap to [-mod/2, mod/2)
  if (d >  (int32_t)(mod / 2u)) d -= (int32_t)mod;
  if (d < -(int32_t)(mod / 2u)) d += (int32_t)mod;

  *prev = (int32_t)cur;
  return d;
}

void Odom4_Init(Odom4_t *o,
                TIM_HandleTypeDef *enc_fl,
                TIM_HandleTypeDef *enc_fr,
                TIM_HandleTypeDef *enc_rl,
                TIM_HandleTypeDef *enc_rr,
                float wheel_radius_m,
                float track_width_m,
                float counts_per_rev)
{
  if (!o) return;

  o->enc_fl = enc_fl;
  o->enc_fr = enc_fr;
  o->enc_rl = enc_rl;
  o->enc_rr = enc_rr;

  o->wheel_radius_m = wheel_radius_m;
  o->track_width_m  = track_width_m;
  o->counts_per_rev = counts_per_rev;

  o->x_m = 0.0f;
  o->y_m = 0.0f;
  o->theta_rad = 0.0f;

  o->v_mps = 0.0f;
  o->w_rps = 0.0f;

  // default signs: +1 (flip to -1 after a quick empirical test if needed)
  o->sign_fl =  1;
  o->sign_fr =  1;
  o->sign_rl =  1;
  o->sign_rr =  1;

  // start encoder timers (safe even if already started)
  if (o->enc_fl) HAL_TIM_Encoder_Start(o->enc_fl, TIM_CHANNEL_ALL);
  if (o->enc_fr) HAL_TIM_Encoder_Start(o->enc_fr, TIM_CHANNEL_ALL);
  if (o->enc_rl) HAL_TIM_Encoder_Start(o->enc_rl, TIM_CHANNEL_ALL);
  if (o->enc_rr) HAL_TIM_Encoder_Start(o->enc_rr, TIM_CHANNEL_ALL);

  // initialize prev counts to current
  o->prev_fl = o->enc_fl ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_fl) : 0;
  o->prev_fr = o->enc_fr ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_fr) : 0;
  o->prev_rl = o->enc_rl ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_rl) : 0;
  o->prev_rr = o->enc_rr ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_rr) : 0;

  o->last_ms = HAL_GetTick();
  o->initialized = 1;
}

void Odom4_Reset(Odom4_t *o)
{
  if (!o || !o->initialized) return;

  o->x_m = 0.0f;
  o->y_m = 0.0f;
  o->theta_rad = 0.0f;

  o->v_mps = 0.0f;
  o->w_rps = 0.0f;

  o->prev_fl = o->enc_fl ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_fl) : 0;
  o->prev_fr = o->enc_fr ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_fr) : 0;
  o->prev_rl = o->enc_rl ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_rl) : 0;
  o->prev_rr = o->enc_rr ? (int32_t)__HAL_TIM_GET_COUNTER(o->enc_rr) : 0;

  o->last_ms = HAL_GetTick();
}

void Odom4_Update(Odom4_t *o, uint32_t now_ms)
{
  if (!o || !o->initialized) return;

  uint32_t dt_ms = now_ms - o->last_ms;
  if (dt_ms == 0u) return;
  o->last_ms = now_ms;

  // encoder deltas (counts)
  int32_t d_fl = o->enc_fl ? delta_mod(o->enc_fl, &o->prev_fl) : 0;
  int32_t d_fr = o->enc_fr ? delta_mod(o->enc_fr, &o->prev_fr) : 0;
  int32_t d_rl = o->enc_rl ? delta_mod(o->enc_rl, &o->prev_rl) : 0;
  int32_t d_rr = o->enc_rr ? delta_mod(o->enc_rr, &o->prev_rr) : 0;

  // apply sign corrections
  d_fl *= (int32_t)o->sign_fl;
  d_fr *= (int32_t)o->sign_fr;
  d_rl *= (int32_t)o->sign_rl;
  d_rr *= (int32_t)o->sign_rr;

  // counts -> meters
  // meters_per_count = (2*pi*R) / counts_per_rev
  float meters_per_count = (2.0f*(float)M_PI*o->wheel_radius_m) / o->counts_per_rev;

  float s_fl = meters_per_count * (float)d_fl;
  float s_fr = meters_per_count * (float)d_fr;
  float s_rl = meters_per_count * (float)d_rl;
  float s_rr = meters_per_count * (float)d_rr;

  // left vs right average
  float s_left  = 0.5f * (s_fl + s_rl);
  float s_right = 0.5f * (s_fr + s_rr);

  // incremental motion
  float ds     = 0.5f * (s_left + s_right);
  float dtheta = (s_right - s_left) / o->track_width_m;

  // midpoint integration
  float theta_mid = o->theta_rad + 0.5f * dtheta;
  o->x_m       += ds * cosf(theta_mid);
  o->y_m       += ds * sinf(theta_mid);
  o->theta_rad  = wrap_pi(o->theta_rad + dtheta);

  // velocities
  float dt = (float)dt_ms * 1e-3f;
  o->v_mps = ds / dt;
  o->w_rps = dtheta / dt;
}

int Odom4_FormatTelemetry(const Odom4_t *o, char *buf, size_t buf_sz)
{
  if (!o || !buf || buf_sz < 8) return -1;
  // "O x y th v w\n"
  // Keep it compact but readable.
  int n = snprintf(buf, buf_sz, "O %.4f %.4f %.4f %.3f %.3f\n",
                   (double)o->x_m, (double)o->y_m, (double)o->theta_rad,
                   (double)o->v_mps, (double)o->w_rps);
  if (n < 0 || (size_t)n >= buf_sz) return -1;
  return n;
}
