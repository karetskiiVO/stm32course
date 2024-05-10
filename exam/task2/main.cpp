#include <stm32f0xx.h>
#include <stm32f051x8.h>
#include <system_stm32f0xx.h>

#include <cstdio>
#include <cmath>

/**
 *
 * Сделать частотомер на таймере(максимальная частота 25.5 kHz), результат измерения 
 * отсылается по USART на компьютер в формате строки.
 * 
 * Распиновка
 * 
 * PB4  -> TIM3CH1  -- ввход сигнала
 * PA9  -> USART1TX -- выход на USART
 * 
 * (c) Karetskii Vlad & Alisa Viktorova, MIPT 2024
 * 
 */

#define OUTPUT_BUFFER_SIZE 32

const uint32_t FreqInPin   = 4;
const uint32_t UsartOutPin = 9;
const uint32_t ApbFreq     = 8'000'000;

enum class PinMode   : uint32_t { 
    INPUT       = 0b00, 
    OUTPUT      = 0b01, 
    ALTERNATIVE = 0b10, 
    ANALOG      = 0b11, 
}; 
enum class SpeedMode : uint32_t { 
    LOW         = 0b00, 
    FAST        = 0b01, 
    VERYFAST    = 0b10, 
}; 
enum class PullMode  : uint32_t { 
    NOPULL      = 0b00, 
    PULLUP      = 0b01, 
    PULLDOWN    = 0b10, 
}; 
 
void configurePin ( 
    GPIO_TypeDef* GPIOx, uint32_t pin, PinMode pinMode, 
    SpeedMode speedMode, PullMode pullMode, uint32_t alternativeFunc);

void USART1TransmitInit (uint32_t baudRate);
void USART1SendString (const char* str);

void TIM3CH1FreqInPinputInit ();

void TIM6Init ();

void SysTickInit ();
void delayms (uint32_t ms);

volatile uint32_t measured = 0;

int main (void) {
    USART1TransmitInit(9600); 
    TIM6Init();
    SysTickInit();
    TIM3CH1FreqInPinputInit();

    NVIC_DisableIRQ(USART1_IRQn);               // выключаем прерывания USART, чтою ничего не ломать

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          // включаем тактирование GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;          // включаем тактирование GPIOB
    /* 
    ** Кофигурируем пины следующим образом
    **
    ** PB4  -> TIM3CH1
    ** PA9  -> USART1TX
    */
    configurePin(GPIOB, FreqInPin,   PinMode::ALTERNATIVE, SpeedMode::LOW, PullMode::NOPULL, 1);
    configurePin(GPIOA, UsartOutPin, PinMode::ALTERNATIVE, SpeedMode::LOW, PullMode::NOPULL, 1);
    
    char answer[OUTPUT_BUFFER_SIZE];            // содаем буфер в который будем записывать ответ

    while (1) {
        delayms(1000);                          // ждем одну секунду, при этом крутимся в прерывании, считая ответ

        sprintf(answer, "%ld Hz\n", measured);
        USART1SendString(answer);               // формируем и отправляем ответ
    }
    
    return 0;
}

void SysTickInit () {
    const uint16_t chunk = 60000;
    SysTick->LOAD = chunk - 1;
    SysTick->VAL  = chunk - 1;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void delayms (uint32_t ms) {
    const uint16_t chunk = 60000;

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

void TIM6Init () {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;         // включаем TIM6

    TIM6->PSC = 8 - 1;                          // ставем предделитель 8 на TIM6
    TIM6->ARR = UINT16_MAX;                     // устанавливаем максимальны диапазон подсчета времени
    
    TIM6->CR1 |= TIM_CR1_CEN;                   // включаем отсчет на TIM6
    while (!(TIM6->SR & TIM_SR_UIF)) {}         // ждем пока таймер обновится и примет новые настройки
}

void USART1TransmitInit (uint32_t baudRate) { 
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;       // включаем тактирование USART1
 
    NVIC_DisableIRQ(USART1_IRQn);               // выключаем на время настройки, для избежаний deadlock'ов
    
    USART1->BRR = ApbFreq / baudRate;           // установили скорость прием-передачи в сек [бод]
    USART1->CR1 |= USART_CR1_TE;                // включаем отправку
    USART1->CR1 |= USART_CR1_UE;                // включаем USART
 
    NVIC_EnableIRQ(USART1_IRQn);                // возвращаем прерывания
}

void USART1SendString (const char* str) {
    if (str == nullptr) return;                 // проверяем на валидность указатель на строку

    for (; *str != '\0'; str++) {               // проходимся по строке, пока не дойдем до терминального
        while(!(USART1->ISR&USART_ISR_TXE)) {}  // дожидаемся, пока USART не отправит предыдущий байт и выставит флаг готовности
        USART1->TDR = *str;                     // загружаем очередной байт данных для отправки
    }
}

void TIM3CH1FreqInPinputInit () {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;         // включаем тактирование на TIM3

    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;            // настраиваем канал как входной, от которого происходит тактирование
    TIM3->CCER  |= TIM_CCER_CC1E;               // включаем первый канал таймера
    TIM3->CCER  |= TIM_CCER_CC1P;               // включаем режим внешенего тактирования
    TIM3->DIER  |= TIM_DIER_CC1IE;              // включаем генерацию прерываний по событиям первого канала
    TIM3->CCMR1 |= TIM_CCMR1_IC1PSC;            // устанавливаем предделитель 8 для таймерв

    NVIC_EnableIRQ(TIM3_IRQn);                  // включаем прерывания по событиям TIM3
    TIM3->CR1   |= TIM_CR1_CEN;                 // включаем счетчик таймера
}

extern "C" void TIM3_IRQHandler (void) {
    if (TIM3->SR & TIM_SR_CC1IF) {              // проверяем, что флаг прерывания по первому каналу поднят 
        static volatile uint16_t start = 0;     // храним время предыдущего вызова
        volatile uint16_t current = TIM6->CNT;  // получаем текущее время по TIM6
        measured = ApbFreq / (current - start); // обновляем время, так как и у TIM3 и у TIM6 стоит предделитель 8, то 8 сокращается 

        TIM3->SR &= ~TIM_SR_CC1IF;              // опускаем флаг, прерывание обработано

        start = current;                        // обновляем время предыдущего вызова
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
