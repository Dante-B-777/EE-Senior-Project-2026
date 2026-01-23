#include "wheels4.h"

/* =========================
   Example pin mapping
   =========================
   PWM (TIM2): PA0/PA1/PA2/PA3 -> CH1..CH4
   DIR pins : PB0/PB1/PB2/PB10

   Encoders:
     W_FL: TIM3 on PA6/PA7 (CH1/CH2)
     W_FR: TIM4 on PB6/PB7
     W_RL: TIM1 on PA8/PA9
     W_RR: TIM8 on PC6/PC7
*/

Wheel_t g_wheels[4] = {
    // W_FL
    { TIM2, 1, GPIOB, GPIO_PIN_0,  true,  TIM3, 0, 0 },
    // W_FR
    { TIM2, 2, GPIOB, GPIO_PIN_1,  true,  TIM4, 0, 0 },
    // W_RL
    { TIM2, 3, GPIOB, GPIO_PIN_2,  true,  TIM1, 0, 0 },
    // W_RR
    { TIM2, 4, GPIOB, GPIO_PIN_10, true,  TIM8, 0, 0 },
};

static inline void pwm_set_ccr(TIM_TypeDef *tim, uint8_t ch, uint32_t val)
{
    if (ch == 1) tim->CCR1 = val;
    else if (ch == 2) tim->CCR2 = val;
    else if (ch == 3) tim->CCR3 = val;
    else if (ch == 4) tim->CCR4 = val;
}

static inline void dir_write(Wheel_t *w, bool forward)
{
    // forward = DIR low (like your current PB0 behavior) if dir_fwd_is_low == true
    bool set_pin = (w->dir_fwd_is_low) ? (!forward) : (forward);
    if (set_pin) w->dir_port->BSRR = w->dir_pin;
    else         w->dir_port->BRR  = w->dir_pin;
}

/* ---------------- GPIO init (DIR + PWM pins + encoder pins) ----------------
   NOTE: This is intentionally a TEMPLATE:
   - You must set MODER/AFR for each PWM + encoder pin you choose.
   - Below we only show DIR pins + TIM2 PWM pins as an example.
*/
void Wheels_GPIO_Init_All(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN
                 |  RCC_AHB2ENR_GPIOBEN
                 |  RCC_AHB2ENR_GPIOCEN;

    /* DIR pins as outputs */
    // PB0, PB1, PB2, PB10
    GPIOB->MODER &= ~(3U<<(0*2));  GPIOB->MODER |=  (1U<<(0*2));
    GPIOB->MODER &= ~(3U<<(1*2));  GPIOB->MODER |=  (1U<<(1*2));
    GPIOB->MODER &= ~(3U<<(2*2));  GPIOB->MODER |=  (1U<<(2*2));
    GPIOB->MODER &= ~(3U<<(10*2)); GPIOB->MODER |=  (1U<<(10*2));

    /* Default forward (DIR low) */
    GPIOB->BRR = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10;

    /* TIM2 PWM pins: PA0..PA3 AF1 */
    for (int pin = 0; pin <= 3; pin++) {
        GPIOA->MODER &= ~(3U << (pin*2));
        GPIOA->MODER |=  (2U << (pin*2));      // AF
        GPIOA->AFR[0] &= ~(0xFU << (pin*4));
        GPIOA->AFR[0] |=  (0x1U << (pin*4));   // AF1 TIM2
    }

    // Encoder pins: configure similarly (AF mappings depend on your chosen timers/pins)
    // Configure PA6/PA7 AF2 for TIM3, PB6/PB7 AF2 for TIM4, etc.
}

/* ---------------- TIM2 PWM 20kHz with 4 channels ---------------- */
void Wheels_PWM_TIM2_Init_20kHz_All4(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->PSC = 0;
    TIM2->ARR = 199;              // 4MHz/200 = 20kHz

    // CH1 PWM1
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M);
    TIM2->CCMR1 |=  (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

    // CH2 PWM1
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M);
    TIM2->CCMR1 |=  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    // CH3 PWM1
    TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M);
    TIM2->CCMR2 |=  (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    // CH4 PWM1
    TIM2->CCMR2 &= ~(TIM_CCMR2_CC4S | TIM_CCMR2_OC4M);
    TIM2->CCMR2 |=  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM2->CCR1 = 0; TIM2->CCR2 = 0; TIM2->CCR3 = 0; TIM2->CCR4 = 0;

    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* ---------------- Encoder init helper (same as your TIM3 setup) ---------------- */
static void encoder_init(TIM_TypeDef *tim, volatile uint32_t *rcc_en_reg, uint32_t rcc_en_mask)
{
    *rcc_en_reg |= rcc_en_mask;

    tim->CR1 &= ~TIM_CR1_CEN;
    tim->PSC = 0;
    tim->ARR = 0xFFFF;

    tim->SMCR &= ~TIM_SMCR_SMS;
    tim->SMCR |=  (3U << TIM_SMCR_SMS_Pos);  // encoder mode 3

    tim->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    tim->CCMR1 |=  (1U << TIM_CCMR1_CC1S_Pos);
    tim->CCMR1 |=  (1U << TIM_CCMR1_CC2S_Pos);

    tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

    tim->CNT = 0;
    tim->CR1 |= TIM_CR1_CEN;
}

void Wheels_Enc_Init_All(void)
{
    // TIM3 (APB1ENR1)
    encoder_init(TIM3, &RCC->APB1ENR1, RCC_APB1ENR1_TIM3EN);
    // TIM4 (APB1ENR1)
    encoder_init(TIM4, &RCC->APB1ENR1, RCC_APB1ENR1_TIM4EN);
    // TIM1 (APB2ENR)
    encoder_init(TIM1, &RCC->APB2ENR,  RCC_APB2ENR_TIM1EN);
    // TIM8 (APB2ENR)
    encoder_init(TIM8, &RCC->APB2ENR,  RCC_APB2ENR_TIM8EN);
}

/* ---------------- Per-wheel speed ---------------- */
void Wheel_SetSignedSpeed(WheelIndex idx, int16_t cmd)
{
    Wheel_t *w = &g_wheels[idx];

    if (cmd > 1000)  cmd = 1000;
    if (cmd < -1000) cmd = -1000;

    bool forward = (cmd >= 0);
    if (cmd < 0) cmd = (int16_t)(-cmd);

    dir_write(w, forward);

    uint32_t duty = ((uint32_t)cmd * (w->pwm_tim->ARR + 1U)) / 1000U;
    if (duty > w->pwm_tim->ARR) duty = w->pwm_tim->ARR;

    pwm_set_ccr(w->pwm_tim, w->pwm_ch, duty);
}

/* ---------------- 32-bit wrap-safe encoder count per wheel ---------------- */
int32_t Wheel_GetCount32(WheelIndex idx)
{
    Wheel_t *w = &g_wheels[idx];

    uint16_t now = (uint16_t)w->enc_tim->CNT;
    int16_t delta = (int16_t)(now - w->enc_prev);
    w->enc_acc += (int32_t)delta;
    w->enc_prev = now;

    return w->enc_acc;
}

/* ---------------- High-level motions (differential drive) ----------------
   Convention:
     Left side:  FL + RL
     Right side: FR + RR
*/
void Drive_Stop(void)
{
    for (int i=0;i<4;i++) Wheel_SetSignedSpeed((WheelIndex)i, 0);
}

void Drive_Forward(int16_t cmd)
{
    Wheel_SetSignedSpeed(W_FL, +cmd);
    Wheel_SetSignedSpeed(W_RL, +cmd);
    Wheel_SetSignedSpeed(W_FR, +cmd);
    Wheel_SetSignedSpeed(W_RR, +cmd);
}

void Drive_Reverse(int16_t cmd)
{
    Wheel_SetSignedSpeed(W_FL, -cmd);
    Wheel_SetSignedSpeed(W_RL, -cmd);
    Wheel_SetSignedSpeed(W_FR, -cmd);
    Wheel_SetSignedSpeed(W_RR, -cmd);
}

void Drive_TurnLeft(int16_t cmd)
{
    // pivot turn: left wheels reverse, right wheels forward
    Wheel_SetSignedSpeed(W_FL, -cmd);
    Wheel_SetSignedSpeed(W_RL, -cmd);
    Wheel_SetSignedSpeed(W_FR, +cmd);
    Wheel_SetSignedSpeed(W_RR, +cmd);
}

void Drive_TurnRight(int16_t cmd)
{
    Wheel_SetSignedSpeed(W_FL, +cmd);
    Wheel_SetSignedSpeed(W_RL, +cmd);
    Wheel_SetSignedSpeed(W_FR, -cmd);
    Wheel_SetSignedSpeed(W_RR, -cmd);
}

