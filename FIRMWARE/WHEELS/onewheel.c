
#include "main.h"      // <-- important: pulls in HAL + device header chain
#include "wheels.h"
#include <stdint.h>    // int16_t, uint16_t, int32_t, uint8_t
#include <stdarg.h>    // va_list, va_start, va_end
#include <stdio.h>     // vsnprintf


/* ============================================================
   MOTOR: PWM on PA0 (TIM2_CH1), DIR on PB0
   ============================================================ */
void Motor_GPIO_Init(void)
{
    /* Enable GPIOA and GPIOB clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    /* DIR: PB0 output */
    GPIOB->MODER &= ~(3U << (0 * 2));
    GPIOB->MODER |=  (1U << (0 * 2));
    GPIOB->OTYPER &= ~(1U << 0);
    GPIOB->PUPDR  &= ~(3U << (0 * 2));
    GPIOB->OSPEEDR &= ~(3U << (0 * 2));
    GPIOB->BRR = (1U << 0); /* default forward */

    /* PWM: PA0 AF1 TIM2_CH1 */
    GPIOA->MODER &= ~(3U << (0 * 2));
    GPIOA->MODER |=  (2U << (0 * 2));
    GPIOA->AFR[0] &= ~(0xFU << (0 * 4));
    GPIOA->AFR[0] |=  (0x1U << (0 * 4)); /* AF1 */
    GPIOA->OTYPER &= ~(1U << 0);
    GPIOA->PUPDR  &= ~(3U << (0 * 2));
    GPIOA->OSPEEDR |= (3U << (0 * 2));
}

void TIM2_PWM_Init_20kHz(void)
{
    /*
      With your SystemClock_Config (MSI range 6, no PLL):
        SYSCLK ~ 4 MHz
        APB1 divider /2 => PCLK1 ~ 2 MHz
        TIM2 clock typically = 2*PCLK1 (APB prescaler > 1) => ~4 MHz

      20 kHz target:
        fPWM = fTIM / ((PSC+1)*(ARR+1))
        PSC=0, ARR=199 => 4MHz/200 = 20kHz
    */
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->PSC = 0;
    TIM2->ARR = 199;

    /* PWM mode 1 on CH1 */
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |=  (6U << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR1 |=  TIM_CCMR1_OC1PE;

    TIM2->CCER |= TIM_CCER_CC1E;

    TIM2->CCR1 = 0; /* start off */
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void Motor_SetSignedSpeed(int16_t cmd)
{
    if (cmd > 1000)  cmd = 1000;
    if (cmd < -1000) cmd = -1000;

    /* DIR: PB0 low=forward, high=reverse (swap if needed) */
    if (cmd >= 0)
    {
        GPIOB->BRR = (1U << 0);
    }
    else
    {
        GPIOB->BSRR = (1U << 0);
        cmd = (int16_t)(-cmd);
    }

    uint32_t duty = ((uint32_t)cmd * (TIM2->ARR + 1U)) / 1000U;
    if (duty > TIM2->ARR) duty = TIM2->ARR;

    TIM2->CCR1 = duty;
}

/* ============================================================
   ENCODER: TIM3 encoder mode on PA6/PA7
   ============================================================ */
void Encoder_GPIO_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    /* PA6/PA7 alternate function */
    GPIOA->MODER &= ~(3U << (6 * 2));
    GPIOA->MODER &= ~(3U << (7 * 2));
    GPIOA->MODER |=  (2U << (6 * 2));
    GPIOA->MODER |=  (2U << (7 * 2));

    /* AF2 for TIM3 on PA6/PA7 */
    GPIOA->AFR[0] &= ~(0xFU << (6 * 4));
    GPIOA->AFR[0] &= ~(0xFU << (7 * 4));
    GPIOA->AFR[0] |=  (0x2U << (6 * 4));
    GPIOA->AFR[0] |=  (0x2U << (7 * 4));

    /* Pull-ups help if encoder outputs are open-collector */
    GPIOA->PUPDR &= ~(3U << (6 * 2));
    GPIOA->PUPDR &= ~(3U << (7 * 2));
    GPIOA->PUPDR |=  (1U << (6 * 2));
    GPIOA->PUPDR |=  (1U << (7 * 2));

    GPIOA->OSPEEDR |= (3U << (6 * 2)) | (3U << (7 * 2));
}

void TIM3_Encoder_Init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;

    /* Encoder mode 3 (count on both TI1 and TI2 edges): SMS=011 */
    TIM3->SMCR &= ~TIM_SMCR_SMS;
    TIM3->SMCR |=  (3U << TIM_SMCR_SMS_Pos);

    /* CC1S=01 (TI1), CC2S=01 (TI2) */
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM3->CCMR1 |=  (1U << TIM_CCMR1_CC1S_Pos);
    TIM3->CCMR1 |=  (1U << TIM_CCMR1_CC2S_Pos);

    /* Non-inverted polarity */
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);

    /* Enable captures */
    TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

    TIM3->CNT = 0;
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* Wrap-safe 32-bit extended count (call regularly) */
int32_t Encoder_GetCount32(void)
{
    static uint16_t prev = 0;
    static int32_t acc = 0;

    uint16_t now = (uint16_t)TIM3->CNT;
    int16_t delta = (int16_t)(now - prev); /* signed handles wrap */
    acc += (int32_t)delta;
    prev = now;

    return acc;
}

/* ============================================================
   LPUART1: Register-level init + print (no float)
   ============================================================ */
void LPUART1_Init_115200_MSI4MHz(void)
{
    /* Enable GPIOG clock */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;

    /* PG7/PG8 AF8 for LPUART1 */
    GPIOG->MODER &= ~(3U << (7 * 2));
    GPIOG->MODER &= ~(3U << (8 * 2));
    GPIOG->MODER |=  (2U << (7 * 2));
    GPIOG->MODER |=  (2U << (8 * 2));

    /* AF: PG7 is AFR[0], PG8 is AFR[1] */
    GPIOG->AFR[0] &= ~(0xFU << (7 * 4));
    GPIOG->AFR[0] |=  (0x8U << (7 * 4)); /* AF8 */

    GPIOG->AFR[1] &= ~(0xFU << ((8 - 8) * 4));
    GPIOG->AFR[1] |=  (0x8U << ((8 - 8) * 4)); /* AF8 */

    GPIOG->OTYPER &= ~((1U << 7) | (1U << 8));
    GPIOG->PUPDR  &= ~(3U << (7 * 2));
    GPIOG->PUPDR  &= ~(3U << (8 * 2));
    GPIOG->OSPEEDR |= (3U << (7 * 2)) | (3U << (8 * 2));

    /* Enable LPUART1 clock */
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;

    /* Select LPUART1 clock source = MSI (01) */
    RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL;
    RCC->CCIPR |=  (1U << RCC_CCIPR_LPUART1SEL_Pos);

    /* Disable UART before config */
    LPUART1->CR1 &= ~USART_CR1_UE;

    /*
      LPUART BRR (oversampling by 256):
        BRR = 256 * fck / baud
      fck = 4,000,000 (MSI range 6), baud=115200:
        BRR ≈ 8889
    */
    LPUART1->BRR = 8889U;

    /* 8N1, enable TX and RX */
    LPUART1->CR1 = 0;
    LPUART1->CR2 = 0;
    LPUART1->CR3 = 0;

    LPUART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    LPUART1->CR1 |= USART_CR1_UE;

    while ((LPUART1->ISR & USART_ISR_TEACK) == 0) {}
    while ((LPUART1->ISR & USART_ISR_REACK) == 0) {}
}

void uart_putc(char c)
{
    while ((LPUART1->ISR & USART_ISR_TXE) == 0) {}
    LPUART1->TDR = (uint8_t)c;
}

void uart_write(const char *s)
{
    while (*s) uart_putc(*s++);
}

void uart_printf(const char *fmt, ...)
{
    char buf[220];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_write(buf);
}

/* ============================================================
   INT-ONLY telemetry (no floats)
   ============================================================ */
int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

void Print_Telemetry_IntOnly(int32_t cnt)
{
    /* Motor rev in milli-rev (rev * 1000) */
    int32_t motor_mrev = (cnt * 1000) / CPR_MOTOR_SHAFT_I;

    /* Wheel rev in milli-rev using exact 19.2 = 192/10 */
    int32_t wheel_mrev = (cnt * 1000 * GR_DEN) / (CPR_MOTOR_SHAFT_I * GR_NUM);

    /* Motor angle in centi-deg (deg * 100) */
    int32_t motor_mod = cnt % CPR_MOTOR_SHAFT_I;
    if (motor_mod < 0) motor_mod += CPR_MOTOR_SHAFT_I;
    int32_t motor_cdeg = (motor_mod * 36000) / CPR_MOTOR_SHAFT_I;

    /* Wheel angle (centi-deg) from wheel fractional rev */
    int32_t wheel_frac_mrev = wheel_mrev % 1000;
    if (wheel_frac_mrev < 0) wheel_frac_mrev += 1000;
    int32_t wheel_cdeg = (wheel_frac_mrev * 36000) / 1000;

    /* Break into printable parts */
    int32_t motor_rev_int  = motor_mrev / 1000;
    int32_t motor_rev_frac = iabs32(motor_mrev % 1000);

    int32_t wheel_rev_int  = wheel_mrev / 1000;
    int32_t wheel_rev_frac = iabs32(wheel_mrev % 1000);

    int32_t motor_deg_int  = motor_cdeg / 100;
    int32_t motor_deg_frac = motor_cdeg % 100;

    int32_t wheel_deg_int  = wheel_cdeg / 100;
    int32_t wheel_deg_frac = wheel_cdeg % 100;

    uart_printf("cnt=%ld | motor: rev=%ld.%03ld ang=%ld.%02ld deg | wheel: rev=%ld.%03ld ang=%ld.%02ld deg\r\n",
        (long)cnt,
        (long)motor_rev_int, (long)motor_rev_frac,
        (long)motor_deg_int, (long)motor_deg_frac,
        (long)wheel_rev_int, (long)wheel_rev_frac,
        (long)wheel_deg_int, (long)wheel_deg_frac
    );
}



