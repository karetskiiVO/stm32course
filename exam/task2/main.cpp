#include <stm32f0xx.h>
#include <stm32f051x8.h>
#include <system_stm32f0xx.h>

#include <cstdio>
#include <cmath>

/**
 * (c) Karetskii Vlad & Alisa Viktorove, MIPT 2024
 * 
 * Задача 2: сделать частотомер на таймере(максимальная частота 25.5 kHz), результат измерения 
 * отсылается по USART на компьютер в формате строки.
 *

/**
 * PB0  -> TIM3CH3
 * 
 * PA9  -> USART1TX
 * PA10 -> USART1RX  
 */

const uint32_t PWMOUTpin  = 0;
const uint32_t USARTINpin = 9;
const uint32_t APBFreq    = 8'000'000;

enum class PinMode : uint32_t { 
    INPUT = 0b00, 
    OUTPUT = 0b01, 
    ALTERNATIVE = 0b10, 
    ANALOG = 0b11, 
}; 
enum class SpeedMode : uint32_t { 
    LOW = 0b00, 
    FAST = 0b01, 
    VERYFAST = 0b10, 
}; 
enum class PullMode : uint32_t { 
    NOPULL = 0b00, 
    PULLUP = 0b01, 
    PULLDOWN = 0b10, 
}; 
 
void configurePin ( 
    GPIO_TypeDef* GPIOx, uint32_t pin, PinMode pinMode, 
    SpeedMode speedMode, PullMode pullMode, uint32_t alternativeFunc);
void USART1TransmitInit (uint32_t baudRate);
void TIM3CH3FreqInputInit ();

void TIM6Init () {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // включаем таймер 6

    TIM6->PSC = 0;                      // таймер 6 на 8'000'000 Гц
    TIM6->ARR = UINT16_MAX;            
    
    TIM6->CR1 |= TIM_CR1_CEN;           // counter enable
    while (!(TIM6->SR & TIM_SR_UIF)) {} // update interupt flag
}

uint16_t chunk = 60000;

void SysTickInit () {
    SysTick->LOAD = chunk - 1;
    SysTick->VAL  = chunk - 1; 
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void delay_ms (uint32_t ms) {
    ms *= 1000;

    for (; ms > chunk; ms -= chunk) {
        SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
        SysTick->VAL = chunk;
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {}
    }

    SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
    SysTick->VAL = ms;
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {}
}

float measured = 0;

void USART1SendString (const char* str);

int main (void) {
    USART1TransmitInit(9600); // включаем usart
    TIM6Init();
    SysTickInit();
    TIM3CH3FreqInputInit();         // включаем pwm на tim3ch3
    NVIC_DisableIRQ(USART1_IRQn);

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // включаем тактирование GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // включаес тактирование GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // включаес тактирование GPIOB

    configurePin(GPIOB, PWMOUTpin,  PinMode::ALTERNATIVE, SpeedMode::LOW, PullMode::NOPULL, 1);
    configurePin(GPIOA, USARTINpin, PinMode::ALTERNATIVE, SpeedMode::LOW, PullMode::NOPULL, 1);

    configurePin(GPIOC, 8, PinMode::OUTPUT, SpeedMode::LOW, PullMode::NOPULL, 0);
    
    char answer[32];

    uint8_t mode = 1;
    while (1) {
        delay_ms(1000);
        
        GPIOC->ODR &= ~(1 << 8);
        GPIOC->ODR |= mode << 8;
        mode = !mode;

        sprintf(answer, "%d Hz\n", (uint32_t)measured);
        USART1SendString(answer);
    }
    
    return 0;
}

void USART1TransmitInit (uint32_t baudRate) { 
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 
 
    NVIC_DisableIRQ(USART1_IRQn); //выключаем на время настройки 
    
    USART1->BRR = APBFreq / baudRate; // установили скорость прием-передачи в сек [бод]
    USART1->CR1 |= USART_CR1_TE;        // включаем прием
    USART1->CR1 |= USART_CR1_UE;        // включаем usart
 
    NVIC_EnableIRQ(USART1_IRQn); 
}

void USART1SendString (const char* str) {
    if (str == nullptr) return;

    for (; *str != '\0'; str++) {
        while(!(USART1->ISR & USART_ISR_TXE)) {}

        SysTick_CTRL_COUNTFLAG_Msk;

        USART1->TDR = *str; 
    }
}

void TIM3CH3FreqInputInit () {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM3->CCER |= TIM_CCER_CC3E;
    TIM3->CCER |= TIM_CCER_CC3P;
    TIM3->DIER |= TIM_DIER_CC3IE;

    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

extern "C" void TIM3_IRQHandler(void) {
    static volatile uint16_t start = UINT16_MAX - TIM6->CNT;
    
    if (TIM3->SR & TIM_SR_CC3IF) {
        volatile uint16_t current = UINT16_MAX - TIM6->CNT;

        measured = float(APBFreq) / (start - current);
        
        TIM3->SR &= ~TIM_SR_CC3IF;

        start = current;
    }
}

void configurePin ( 
    GPIO_TypeDef* GPIOx, uint32_t pin, PinMode pinMode, 
    SpeedMode speedMode, PullMode pullMode, uint32_t alternativeFunc) { 
     
    GPIOx->MODER &= ~(0b11 << (2 * pin)); 
    GPIOx->MODER |= uint32_t(pinMode) << (2 * pin); 
 
    GPIOx->OSPEEDR &= ~(0b11 << (2 * pin)); 
    GPIOx->OSPEEDR |= uint32_t(speedMode) << (2 * pin); 
 
    GPIOx->PUPDR &= ~(0b11 << (2 * pin)); 
    GPIOx->PUPDR |= uint32_t(pullMode) << (2 * pin); 
 
    GPIOx->AFR[pin / 8] &= ~(0x0ful << ((pin % 8ul) * 4ul)); 
    GPIOx->AFR[pin / 8] |= uint32_t(alternativeFunc & 0x0ful) << ((pin % 8ul) * 4ul);  
}
