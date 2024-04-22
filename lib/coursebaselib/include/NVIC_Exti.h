/**
 ******************************************************************************
 *          NVIC_Exti.h
 *          A.Chiffa Dev.
 *          Header file for NVIC and EXTI modules.
 *          For STM32F415RG
 *          V1.0
 * @attention
 *          Redistributions of source code must retain the above copyright
 *notice, this list of conditions and the following disclaimer. Commercial use
 *is possible only by agreement with the developers.
 * @Mail    ArsChiffa@gmail.com
 *
 ******************************************************************************
 */
#ifndef NVIC_Exti_h
#define NVIC_Exti_h

#include "GPIO.h"
#include "stm32f0xx.h"

//------------------------------------------------------------------------------
//                            EXTI LINES definitions
//------------------------------------------------------------------------------

#define EXTI_L0 EXTI_IMR_MR0    // PIN0
#define EXTI_L1 EXTI_IMR_MR1    // PIN1
#define EXTI_L2 EXTI_IMR_MR2    // PIN2
#define EXTI_L3 EXTI_IMR_MR3    // PIN3
#define EXTI_L4 EXTI_IMR_MR4    // PIN4
#define EXTI_L5 EXTI_IMR_MR5    // PIN5
#define EXTI_L6 EXTI_IMR_MR6    // PIN6
#define EXTI_L7 EXTI_IMR_MR7    // PIN7
#define EXTI_L8 EXTI_IMR_MR8    // PIN8
#define EXTI_L9 EXTI_IMR_MR9    // PIN9
#define EXTI_L10 EXTI_IMR_MR10  // PIN10
#define EXTI_L11 EXTI_IMR_MR11  // PIN11
#define EXTI_L12 EXTI_IMR_MR12  // PIN12
#define EXTI_L13 EXTI_IMR_MR13  // PIN13
#define EXTI_L14 EXTI_IMR_MR14  // PIN14
#define EXTI_L15 EXTI_IMR_MR15  // PIN15
#define EXTI_L16 EXTI_IMR_MR16  // PVD output
#define EXTI_L17 EXTI_IMR_MR17  // RTC Alarm event
#define EXTI_L18 EXTI_IMR_MR18  // USB OTG FS Wakeup event
#define EXTI_L19 EXTI_IMR_MR19  // Ethernet Wakeup event
#define EXTI_L20 EXTI_IMR_MR20  // USB OTG HS (configured in FS) Wakeup event
#define EXTI_L21 EXTI_IMR_MR21  // RTC Tamper and TimeStamp events
#define EXTI_L22 EXTI_IMR_MR22  // RTC Wakeup event

//------------------------------------------------------------------------------
//                            EXTI parametrs definitions
//------------------------------------------------------------------------------

#define EXTI_FALLING  0x01      // triggering by falling edge of signal
#define EXTI_RISING   0x02      // triggering by rising edge of signal
#define EXTI_BOTHEDGE 0x03      // triggering by doth edges of signal
#define EXTI_NOTRIG   0x00      // NO triggering by signal

//------------------------------------------------------------------------------
//                            EXTI MACRO definitions
//------------------------------------------------------------------------------
/*!< In EXTI no need to definition of Pin's alternate function  */
// config triggering edge in line
#define EXTI_FALLING_EDGE(LINE) \
    {                           \
        EXTI->FTSR |= LINE;     \
        EXTI->RTSR &= ~LINE;    \
    }
#define EXTI_RISING_EDGE(LINE) \
    {                          \
        EXTI->FTSR &= ~LINE;   \
        EXTI->RTSR |= LINE;    \
    }
#define EXTI_BOTH_EDGE(LINE) \
    {                        \
        EXTI->FTSR |= LINE;  \
        EXTI->RTSR |= LINE;  \
    }
#define EXTI_NO_TRIGGERING(LINE) \
    {                            \
        EXTI->FTSR &= ~LINE;     \
        EXTI->RTSR &= ~LINE;     \
    }

// Enable/disabe EVENT or Interrupt
#define EXTI_ENABLE_EVENT(LINE) \
    {                           \
        EXTI->EMR |= LINE;      \
        EXTI->IMR &= ~LINE;     \
    }
#define EXTI_ENABLE_IT(LINE) \
    {                        \
        EXTI->EMR &= ~LINE;  \
        EXTI->IMR |= LINE;   \
    }
#define EXTI_DISABLE_EVENT(LINE) \
    { EXTI->EMR &= ~LINE; }
#define EXTI_DISABLE_IT(LINE) \
    { EXTI->IMR &= ~LINE; }

#define EXTI_CLOCK \
    { RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; }  // Clock SYSCFGEN which clocks EXTI
#define EXTI_SOFTWARE_IT_EVENT(LINE) \
    { EXTI->SWIER |= LINE; }                        // Trig EXTI line by software
#define EXTI_IS_IT_PENDING(LINE) (EXTI->PR & LINE)  // Check pending interrupt in EXTI module
#define EXTI_CLEAR_FLAG(LINE) \
    { EXTI->PR = LINE; }  // Clear pending flag in EXTI module

// Maps pin to corresponding line
#define EXTI_MAP_PIN(PIN) \
    { SYSCFG->EXTICR[((POSITION_VAL(PIN & 0xFFFF)) / 4)] |= ((PIN >> 26) << (((POSITION_VAL(PIN & 0xFFFF)) % 4) * 4)); }

//------------------------------------------------------------------------------
//                            EXTI_Pin2Event (function)
//------------------------------------------------------------------------------

void EXTI_Pin2Event (uint32_t pin, uint32_t trigedge) {
    EXTI_CLOCK
    if (trigedge == EXTI_FALLING)
        EXTI_FALLING_EDGE(pin & 0xFFFFUL)
    else if (trigedge == EXTI_RISING)
        EXTI_RISING_EDGE(pin & 0xFFFFUL)
    else if (trigedge == EXTI_BOTHEDGE)
        EXTI_BOTH_EDGE(pin & 0xFFFFUL)
    else
        EXTI_NO_TRIGGERING(pin & 0xFFFFUL)
    EXTI_MAP_PIN(pin)
    EXTI_CLEAR_FLAG(pin & 0xFFFFUL)
    EXTI_ENABLE_EVENT(pin & 0xFFFFUL)
}
/*!   EXTI_Pin2Event( Pin, TrigEdge ) Configure EXTI to generate event
 *    or interrupt by defined edge on defined Pin  & Clocks EXTI
 *    @Arguments for  EXTI_Pin2Event( Pin, TrigEdge ):
 *    Pin ref. to GPIO.h         EXTI_FALLING
 *                               EXTI_RISING
 *                               EXTI_BOTHEDGE
 *                               EXTI_NOTRIG
 */

//------------------------------------------------------------------------------
//                            EXTI_Pin2IT (function)
//-----------------------------------------------------------------------------

void EXTI_Pin2IT (uint32_t pin, uint32_t trigedge) {
    EXTI_CLOCK
    if (trigedge == EXTI_FALLING)
        EXTI_FALLING_EDGE(pin & 0xFFFFUL)
    else if (trigedge == EXTI_RISING)
        EXTI_RISING_EDGE(pin & 0xFFFFUL)
    else if (trigedge == EXTI_BOTHEDGE)
        EXTI_BOTH_EDGE(pin & 0xFFFFUL)
    else
        EXTI_NO_TRIGGERING(pin & 0xFFFFUL)
    EXTI_MAP_PIN(pin)
    EXTI_CLEAR_FLAG(pin & 0xFFFFUL)
    EXTI_ENABLE_IT(pin & 0xFFFFUL)
}
/*!   EXTI_Pin2IT( Pin, TrigEdge ) Configure EXTI to generate interrupt
 *    by defined edge on defined Pin  & Clocks EXTI
 *    @Arguments for  EXTI_Pin2IT( Pin, TrigEdge ):
 *    Pin ref. to GPIO.h     EXTI_FALLING
 *                           EXTI_RISING
 *                           EXTI_BOTHEDGE
 *                           EXTI_NOTRIG
 */

/** NVIC functions defined in core_cm0.h */
// IRQ is a position in vector table(0=WWDG)
extern void NVIC_EnableIRQ         (IRQn_Type IRQn);                        // Enables an interrupt or exception.
extern void NVIC_DisableIRQ        (IRQn_Type IRQn);                        // Disables an interrupt or exception.
extern void NVIC_SetPendingIRQ     (IRQn_Type IRQn);                        // Sets the pending status of interrupt or exception
extern void NVIC_ClearPendingIRQ   (IRQn_Type IRQn);                        // Clears the pending status of interrupt or exception
extern void __disable_irq          (void);                                  // Global Interrupts Disable
extern void __enable_irq           (void);                                  // Global Interrupts Enable
extern void NVIC_SystemReset       (void);                                  // System reset
extern void NVIC_SetPriority       (IRQn_Type IRQn, uint32_t priority);     // Sets the priority of an interrupt or exception.
extern uint32_t NVIC_GetPriority   (IRQn_Type IRQn);                        // This function return the current priority level.
extern uint32_t NVIC_GetPendingIRQ (IRQn_Type IRQn);                        // Reads the pending status of interrupt or exception.
#endif