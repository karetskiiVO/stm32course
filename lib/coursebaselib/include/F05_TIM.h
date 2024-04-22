#ifndef F05_TIM_h
#define F05_TIM_h

#include "F05_RCC.h"
#include "GPIO.h"
#include "stm32f0xx.h"

//------------------------------------------------------------------------------
//                            TIM PARAMETERS definitions
//------------------------------------------------------------------------------

#define CAPTURE_MODE_NOPSC                                                                           \
    (0x00UL)                       /* no prescaler, capture is done each time an edge is detected on \
                                      the capture input */
#define CAPTURE_MODE_PSC2 (0x01UL) /* capture is done once every 2 events */
#define CAPTURE_MODE_PSC4 (0x10UL) /* capture is done once every 4 events */
#define CAPTURE_MODE_PSC8 (0x11UL) /* capture is done once every 8 events */

#define COMPARE_MODE_FROZEN (0x00UL) /* There are no effect on the outputs. */
#define COMPARE_MODE_TOGGLE (0x03UL) /* Toggle - OC1REF toggles when TIMx_CNT = TIMx_CCR1. */
#define COMPARE_MODE_PWM1                                                        \
    (0x06UL) /* PWM mode 1 - Channel 1 is active as long as TIMx_CNT < TIMx_CCR1 \
                else inactive. */
#define COMPARE_MODE_PWM2                                                \
    (0x07UL) /* PWM mode 2 - Channel 1 is inactive as long as TIMx_CNT < \
                TIMx_CCR1 else active. */

//------------------------------------------------------------------------------
//                            TIM MACROS definitions
//------------------------------------------------------------------------------

#define COUNTER_ENABLE(TIMx) \
    { TIMx->CR1 |= TIM_CR1_CEN; }
#define COUNTER_DISABLE(TIMx) \
    { TIMx->CR1 &= ~TIM_CR1_CEN; }
#define ONE_PULSE_MODE_SET(TIMx) \
    { TIMx->CR1 |= TIM_CR1_OPM; }
#define ONE_PULSE_MODE_RESET(TIMx) \
    { TIMx->CR1 &= ~TIM_CR1_OPM; }
#define UPCOUNTER(TIMx) \
    { TIMx->CR1 &= ~TIM_CR1_DIR; }
#define DOWNCOUNTER(TIMx) \
    { TIMx->CR1 |= TIM_CR1_DIR; }

/* ARR - auto-reload register */
#define ARR_PRELOAD_ENABLE(TIMx) \
    { TIMx->CR1 |= TIM_CR1_ARPE; }
#define ARR_PRELOAD_DISABLE(TIMx) \
    { TIMx->CR1 &= ~TIM_CR1_ARPE; }

/*! before counter enabling generate update to latch preloded */
#define UPDATE_GENERATE(TIMx) \
    { TIMx->EGR = TIM_EGR_UG; }
#define UPDATE_IT_ENABLE(TIMx) \
    { TIMx->DIER |= TIM_DIER_UIE; }
#define UPDATE_IT_DISABLE(TIMx) \
    { TIMx->DIER &= ~TIM_DIER_UIE; }
#define UPDATE_CLEAR_FLAG(TIMx) \
    { TIMx->SR &= ~TIM_SR_UIF; }

/* CC - Capture Compare abbreviation */

/*! @FILTERING factor should be in range [0x0;0xF]
The higher the number, the stronger the filtering */
/* Channel 1*/
#define CC1_ENABLE(TIMx) \
    { TIMx->CCER |= TIM_CCER_CC1E; }
#define CC1_DISABLE(TIMx) \
    { TIMx->CCER &= ~TIM_CCER_CC1E; }
#define CC1_IT_ENABLE(TIMx) \
    { TIMx->DIER |= TIM_DIER_CC1IE; }
#define CC1_IT_DISABLE(TIMx) \
    { TIMx->DIER &= ~TIM_DIER_CC1IE; }
#define CC1_CLEAR_FLAG(TIMx) \
    { TIMx->SR &= ~TIM_SR_CC1IF; }

#define CC1_CAPTURE_MODE(CAPTURE_MODE)                                   \
    {                                                                    \
        TIMx->CCMR1 &= ~TIM_CCMR1_CC1S;                                  \
        TIMx->CCMR1 |= TIM_CCMR1_CC1S_0;                                 \
        TIMx->CCMR1 &= ~TIM_CCMR1_IC1PSC;                                \
        TIMx->CCMR1 |= ((CAPTURE_MODE & 0x3UL) << TIM_CCMR1_IC1PSC_Pos); \
    }

#define CC1_COMPARE_MODE(COMPARE_MODE)                                  \
    {                                                                   \
        TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                                 \
        TIMx->CCMR1 &= ~TIM_CCMR1_CC1S;                                 \
        TIMx->CCMR1 |= ((COMPARE_MODE & 0x07UL) << TIM_CCMR1_OC1M_Pos); \
    }

#define CC1_INPUT_FILTER(FILTERING)                               \
    {                                                             \
        TIMx->CCMR1 &= ~TIM_CCMR1_IC1F;                           \
        TIMx->CCMR1 |= ((FILTERING & 0xF) << TIM_CCMR1_IC1F_Pos); \
    }

/* Channel 2*/
#define CC2_ENABLE(TIMx) \
    { TIMx->CCER |= TIM_CCER_CC2E; }
#define CC2_DISABLE(TIMx) \
    { TIMx->CCER &= ~TIM_CCER_CC2E; }
#define CC2_IT_ENABLE(TIMx) \
    { TIMx->DIER |= TIM_DIER_CC2IE; }
#define CC2_IT_DISABLE(TIMx) \
    { TIMx->DIER &= ~TIM_DIER_CC2IE; }
#define CC2_CLEAR_FLAG(TIMx) \
    { TIMx->SR &= ~TIM_SR_CC2IF; }

#define CC2_CAPTURE_MODE(CAPTURE_MODE)                                   \
    {                                                                    \
        TIMx->CCMR1 &= ~TIM_CCMR1_CC2S;                                  \
        TIMx->CCMR1 |= TIM_CCMR1_CC2S_0;                                 \
        TIMx->CCMR1 &= ~TIM_CCMR1_IC2PSC;                                \
        TIMx->CCMR1 |= ((CAPTURE_MODE & 0x3UL) << TIM_CCMR1_IC2PSC_Pos); \
    }

#define CC2_COMPARE_MODE(COMPARE_MODE)                                  \
    {                                                                   \
        TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                                 \
        TIMx->CCMR1 &= ~TIM_CCMR1_CC2S;                                 \
        TIMx->CCMR1 |= ((COMPARE_MODE & 0x07UL) << TIM_CCMR1_OC2M_Pos); \
    }

#define CC2_INPUT_FILTER(FILTERING)                               \
    {                                                             \
        TIMx->CCMR1 &= ~TIM_CCMR1_IC2F;                           \
        TIMx->CCMR1 |= ((FILTERING & 0xF) << TIM_CCMR1_IC2F_Pos); \
    }

int16_t TIM_Encoder16GetValue(TIM_TypeDef* TIMx) {
    return ((int16_t)(TIMx->CNT) / 4);
}

int32_t TIM_Encoder32GetValue(TIM_TypeDef* TIMx) {
    return ((int32_t)(TIMx->CNT) / 4);
}

void TIM_EncoderInit(TIM_TypeDef* TIMx) {
    /*! Config your @PINs pull down with corresponding AF previously*/
    RCC_TimClock(TIMx);
    /* Time Base init*/
    TIMx->ARR = 0xFFFFUL;  // Auto-reload
    TIMx->PSC = 0x0;       // Prescaler
    UPDATE_GENERATE(TIMx)
    /* Channels init*/
    CC1_CAPTURE_MODE(CAPTURE_MODE_NOPSC)  // CH1 Capture mode without prescaler
    CC2_CAPTURE_MODE(CAPTURE_MODE_NOPSC)  // CH2 Capture mode without prescaler

    TIMx->SMCR &= ~TIM_SMCR_SMS;                    // Clear SMS bits
    TIMx->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;  // Set SMS bits to encoder mode 3

    CC1_INPUT_FILTER(0xFUL)  // Set CH1 filtering
    CC2_INPUT_FILTER(0xFUL)  // Set CH2 filtering
    TIMx->CNT = 0x00;        // Counter register
    COUNTER_ENABLE(TIMx)
}

void TIM_Freq_Init (TIM_TypeDef* TIMx, uint32_t Frequency) {
    RCC_TimClock(TIMx);

    TIMx->PSC = (TIMClock / 0xFFFF) / Frequency;
    TIMx->ARR = (TIMClock / (TIMx->PSC + 1UL)) / Frequency;

    ARR_PRELOAD_ENABLE(TIMx)  // Turn on the ARR preload
}

void CH1_PWM_Init (TIM_TypeDef* TIMx, float Duty) {
    /*! Config your @PINs with corresponding AF previously*/
    CC1_COMPARE_MODE(COMPARE_MODE_PWM1)           // PWM mode 1
    TIMx->CCMR1 |= TIM_CCMR1_OC1PE;               // Turn on CCRx preload
    TIMx->CCR1 = (uint32_t)(Duty * (TIMx->ARR));  // Set Duty
    CC1_ENABLE(TIMx)                              // Turn on the channel
    COUNTER_ENABLE(TIMx)
}

#endif