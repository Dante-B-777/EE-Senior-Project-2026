#include "wheels4.h"
#include <string.h>
#include <stdio.h>

#define LINE_BUF_SZ 64
#define DRIVE_TIMEOUT_MS 250

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t ch;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  // Your earlier behavior: forward = DIR low
  uint8_t dir_fwd_is_low;
  int16_t cmd;
} Motor4_t;

static UART_HandleTypeDef *g_huart = NULL;
static Motor4_t g_m[4];

static volatile char g_line[LINE_BUF_SZ];
static volatile int  g_len = 0;
static volatile int  g_ready = 0;

static uint32_t g_last_ms = 0;

static int clamp_i(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static uint32_t uabs_i32(int32_t x) { return (x < 0) ? (uint32_t)(-x) : (uint32_t)x; }

static void motor_apply(Motor4_t *m, int16_t cmd)
{
  cmd = (int16_t)clamp_i(cmd, -1000, 1000);
  m->cmd = cmd;

  uint8_t forward = (cmd >= 0) ? 1 : 0;

  // DIR write
  // if forward means DIR low, then "forward=1" -> write RESET
  GPIO_PinState dir_state;
  if (m->dir_fwd_is_low) {
    dir_state = forward ? GPIO_PIN_RESET : GPIO_PIN_SET;
  } else {
    dir_state = forward ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(m->dir_port, m->dir_pin, dir_state);

  // PWM duty
  uint32_t mag = uabs_i32((int32_t)cmd);
  if (mag > 1000) mag = 1000;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->htim);
  uint32_t ccr = ((arr + 1u) * mag) / 1000u;
  if (ccr > arr) ccr = arr;

  __HAL_TIM_SET_COMPARE(m->htim, m->ch, ccr);
}

void Wheels4_Init(UART_HandleTypeDef *huart)
{
  g_huart = huart;

  memset((void*)g_line, 0, sizeof(g_line));
  g_len = 0;
  g_ready = 0;

  for (int i=0;i<4;i++) {
    g_m[i].htim = NULL;
    g_m[i].ch = 0;
    g_m[i].dir_port = NULL;
    g_m[i].dir_pin = 0;
    g_m[i].dir_fwd_is_low = 1; // match your original PB0 behavior
    g_m[i].cmd = 0;
  }

  g_last_ms = HAL_GetTick();
}

void Wheels4_ConfigAll(
    TIM_HandleTypeDef *htim2,
    GPIO_TypeDef *dirFL_port, uint16_t dirFL_pin,
    GPIO_TypeDef *dirFR_port, uint16_t dirFR_pin,
    GPIO_TypeDef *dirRL_port, uint16_t dirRL_pin,
    GPIO_TypeDef *dirRR_port, uint16_t dirRR_pin)
{
  // PWM channels on TIM2
  g_m[W_FL] = (Motor4_t){ htim2, TIM_CHANNEL_1, dirFL_port, dirFL_pin, 1, 0 };
  g_m[W_FR] = (Motor4_t){ htim2, TIM_CHANNEL_2, dirFR_port, dirFR_pin, 1, 0 };
  g_m[W_RL] = (Motor4_t){ htim2, TIM_CHANNEL_3, dirRL_port, dirRL_pin, 1, 0 };
  g_m[W_RR] = (Motor4_t){ htim2, TIM_CHANNEL_4, dirRR_port, dirRR_pin, 1, 0 };

  // Start PWM on all 4 channels
  HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);

  Wheels4_Stop();
}

void Wheels4_OnRxByte(uint8_t b)
{
  char c = (char)b;
  if (g_ready) return;
  if (c == '\r') return;

  if (c == '\n' || g_len >= (LINE_BUF_SZ - 1)) {
    g_line[g_len] = '\0';
    g_ready = 1;
    return;
  }
  g_line[g_len++] = c;
}

void Wheel_SetSignedSpeed(WheelIndex idx, int16_t cmd)
{
  if ((int)idx < 0 || (int)idx > 3) return;
  if (!g_m[idx].htim || !g_m[idx].dir_port) return;
  motor_apply(&g_m[idx], cmd);
}

void Wheels4_SetAll(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
  Wheel_SetSignedSpeed(W_FL, fl);
  Wheel_SetSignedSpeed(W_FR, fr);
  Wheel_SetSignedSpeed(W_RL, rl);
  Wheel_SetSignedSpeed(W_RR, rr);
}

void Wheels4_Stop(void)
{
  Wheels4_SetAll(0,0,0,0);
  g_last_ms = HAL_GetTick();
}

static void handle_line(const char *s, uint32_t now_ms)
{
  if (!s || !s[0]) return;

  if (s[0] == 'M') {
    int a,b,c,d;
    if (sscanf(s, "M %d %d %d %d", &a, &b, &c, &d) == 4) {
      a = clamp_i(a, -1000, 1000);
      b = clamp_i(b, -1000, 1000);
      c = clamp_i(c, -1000, 1000);
      d = clamp_i(d, -1000, 1000);
      Wheels4_SetAll((int16_t)a,(int16_t)b,(int16_t)c,(int16_t)d);
      g_last_ms = now_ms;
    }
    return;
  }

  if (s[0] == 'X') {
    Wheels4_Stop();
    return;
  }
}

void Wheels4_Task(uint32_t now_ms)
{
  if ((now_ms - g_last_ms) > DRIVE_TIMEOUT_MS) {
    Wheels4_Stop();
  }

  if (g_ready) {
    char local[LINE_BUF_SZ];

    __disable_irq();
    int n = g_len;
    if (n >= LINE_BUF_SZ) n = LINE_BUF_SZ - 1;
    memcpy(local, (const void*)g_line, (size_t)n);
    local[n] = '\0';
    g_len = 0;
    g_ready = 0;
    __enable_irq();

    handle_line(local, now_ms);
  }
}
