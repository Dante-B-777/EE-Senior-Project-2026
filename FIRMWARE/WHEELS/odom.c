/**
 * @file    odom4.c
 * @brief   Four-wheel differential-drive odometry estimation module
 *
 * This file implements planar odometry for a four-wheel skid-steer robot
 * using incremental quadrature encoder feedback from each wheel.
 *
 * The robot pose is estimated in a 2D world frame (x, y, θ), where:
 *   - x and y represent the robot’s position in meters
 *   - θ represents the robot’s heading in radians
 *   - the initial pose (x = 0, y = 0, θ = 0) corresponds to the robot’s
 *     position and orientation at the moment logging begins or when
 *     Odom4_Reset() is called
 *
 * Encoder counts from the front-left (FL), front-right (FR),
 * rear-left (RL), and rear-right (RR) wheels are read using STM32
 * hardware timers configured in Encoder Interface mode. Wheel travel
 * distances are computed from encoder count deltas using known wheel
 * geometry and encoder resolution.
 *
 * For odometry computation, the robot is modeled as a differential-drive
 * system by averaging the left-side wheels (FL, RL) and right-side wheels
 * (FR, RR). Linear and angular displacements are then integrated using a
 * midpoint (semi-implicit) integration method to improve accuracy during
 * turning maneuvers.
 *
 * Assumptions:
 *   - Planar motion (no slip, no vertical motion)
 *   - Identical wheel radii for all four wheels
 *   - Left and right wheels remain parallel
 *   - Encoder timers are updated frequently enough to avoid overflow
 *
 * Robot parameters used in this project:
 *   - Wheel radius: 52 mm (0.052 m)
 *   - Track width: 415 mm (0.415 m)
 *   - Encoder resolution: ~537.7 pulses per wheel revolution at the
 *     gearbox output, with 4x quadrature decoding
 *
 * This module is intended to be called periodically (e.g., 50–200 Hz)
 * from the main control loop. The resulting pose estimate may be used
 * for navigation, control, telemetry, or sensor fusion.
 *
 * @author
 * Dante Benedetti
 *
 * @date
 * 2026
 */

#include "odom4.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#define WHEEL_RADIUS_M   0.052f
#define TRACK_WIDTH_M    0.415f
#define COUNTS_PER_REV   (537.7f * 4.0f)   // encoder mode usually counts 4x
#endif

static float wrap_pi(float a) {
  while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
  while (a < -(float)M_PI) a += 2.0f*(float)M_PI;
  return a;
}

// Read counter and compute delta robustly for 16-bit encoder timers.
// If you use a 32-bit timer, this still works as long as you update often enough.
static inline int32_t delta_from_16bit(TIM_HandleTypeDef *htim, int32_t *prev) {
  int16_t cur16 = (int16_t)__HAL_TIM_GET_COUNTER(htim);
  int16_t prev16 = (int16_t)(*prev);
  int16_t d16 = (int16_t)(cur16 - prev16);     // handles wrap automatically for 16-bit
  *prev = (int32_t)cur16;
  return (int32_t)d16;
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

  // start encoder timers if not already started elsewhere
  // (safe to call even if you started them in main)
  HAL_TIM_Encoder_Start(o->enc_fl, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(o->enc_fr, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(o->enc_rl, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(o->enc_rr, TIM_CHANNEL_ALL);

  // zero previous counts to current counter values
  o->prev_fl = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_fl);
  o->prev_fr = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_fr);
  o->prev_rl = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_rl);
  o->prev_rr = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_rr);

  o->last_ms = HAL_GetTick();
  o->initialized = 1;
}

void Odom4_Reset(Odom4_t *o)
{
  o->x_m = 0.0f;
  o->y_m = 0.0f;
  o->theta_rad = 0.0f;

  // re-zero delta accumulation to “now”
  o->prev_fl = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_fl);
  o->prev_fr = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_fr);
  o->prev_rl = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_rl);
  o->prev_rr = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(o->enc_rr);

  o->last_ms = HAL_GetTick();
}

void Odom4_Update(Odom4_t *o, uint32_t now_ms)
{
  if (!o->initialized) return;

  // dt is optional for pose integration, but it’s nice to keep for debugging/velocity later
  uint32_t dt_ms = now_ms - o->last_ms;
  if (dt_ms == 0) return;
  o->last_ms = now_ms;

  // encoder count deltas
  int32_t d_fl = delta_from_16bit(o->enc_fl, &o->prev_fl);
  int32_t d_fr = delta_from_16bit(o->enc_fr, &o->prev_fr);
  int32_t d_rl = delta_from_16bit(o->enc_rl, &o->prev_rl);
  int32_t d_rr = delta_from_16bit(o->enc_rr, &o->prev_rr);

  // Convert counts -> meters (per wheel)
  float meters_per_count = (2.0f*(float)M_PI*o->wheel_radius_m) / o->counts_per_rev;

  float s_fl = meters_per_count * (float)d_fl;
  float s_fr = meters_per_count * (float)d_fr;
  float s_rl = meters_per_count * (float)d_rl;
  float s_rr = meters_per_count * (float)d_rr;

  // Skid-steer approximation: average left vs average right
  float s_left  = 0.5f * (s_fl + s_rl);
  float s_right = 0.5f * (s_fr + s_rr);

  float ds     = 0.5f * (s_left + s_right);
  float dtheta = (s_right - s_left) / o->track_width_m;

  // Midpoint integration (better than naive Euler for turning)
  float theta_mid = o->theta_rad + 0.5f * dtheta;
  o->x_m     += ds * cosf(theta_mid);
  o->y_m     += ds * sinf(theta_mid);
  o->theta_rad = wrap_pi(o->theta_rad + dtheta);
}
