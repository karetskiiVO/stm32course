#include <stm32f0xx.h>
#include <stm32f051x8.h>
#include <system_stm32f0xx.h>

/**
 *
 * На микроконтроллер присылается 9-битное число, первый бит(LSB) указывает, передаётся частота(1) 
 * или скважность(0). Остальные 8 бит, несут собственно частоту или скважность (0х00 соответствует 
 * 0Hz или Duty 0%, 0xFF соответствует 25.5kHz или Duty 100%)
 * 
 * Распиновка
 * 
 * PB0  -> TIM3CH3  -- вывод PWM
 * PA10 -> USART1RX -- ввод USART
 * 
 * (c) Karetskii Vlad & Alisa Viktorove, MIPT 2024
 * 
 */

const uint32_t PWMOUTpin  = 0;
const uint32_t USARTINpin = 10;
const uint32_t APBFreq    = 8'000'000;

enum class PinMode   : uint32_t { 
    INPUT       = 0b00, 
    OUTPUT      = 0b01, 
    ALTERNATIVE = 0b10, 
    ANALOG      = 0b11, 
}; 
enum class SpeedMode : uint32_t { 
    LOW      = 0b00, 
    FAST     = 0b01, 
    VERYFAST = 0b10, 
}; 
enum class PullMode  : uint32_t { 
    NOPULL   = 0b00, 
    PULLUP   = 0b01, 
    PULLDOWN = 0b10, 
}; 
 
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

void TIM3CH3PWMInit ();
void TIM3CH3PWMConfigure (uint32_t freq, uint8_t rate);
void USART1ReciveInit (uint32_t baudRate);

int main (void) {
    USART1ReciveInit(9600);   
    TIM3CH3PWMInit();

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          // включаем тактирование GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;          // включаес тактирование GPIOB

    /* 
    ** Кофигурируем пины следующим образом
    **
    ** PB0  -> TIM3CH3
    ** PA10 -> USART1RX
    */
    configurePin(GPIOB, PWMOUTpin,  PinMode::ALTERNATIVE, SpeedMode::LOW, PullMode::NOPULL, 1);
    configurePin(GPIOA, USARTINpin, PinMode::ALTERNATIVE, SpeedMode::LOW, PullMode::NOPULL, 1);

    uint32_t freq = 0, rate = 0;                // переменные текущей частоты и скважности 

    TIM3CH3PWMConfigure(freq, rate);

    while (1) {
        uint8_t val, mode;                      // буферные переменные для приема

        while (!(USART1->ISR&USART_ISR_RXNE));  // ждем, пока USART1 примет первый байт
        mode = USART1->RDR;                     // cчитывам первый, управляющий байт     
        USART1->ISR &= ~USART_ISR_RXNE;         // обнуляем флаг приема, для того чтобы USART1 мог считывать дальше
        
        while (!(USART1->ISR&USART_ISR_RXNE));  // ждем, пока USART1 примет второй байт
        val = USART1->RDR;                      // считываем второй байт со значением
        USART1->ISR &= ~USART_ISR_RXNE;         // обнуляем флаг приема, для того чтобы USART1 мог считывать дальше

        switch (mode & 0b01) {
        case 0:
            rate = val;
            break;
        case 1:
            freq = val; 
            break;
        }
        
        TIM3CH3PWMConfigure(freq, rate);
    }

    return 0;
}

void USART1ReciveInit (uint32_t baudRate) { 
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;       // включаем тактирование для USART1
 
    NVIC_DisableIRQ(USART1_IRQn);               // выключаем на время настройки, для избежаний deadlock'ов
    

    USART1->BRR  = APBFreq / baudRate;          // установили скорость прием-передачи в сек [бод]
    USART1->CR1 |= USART_CR1_RE;                // включаем прием
    USART1->CR1 |= USART_CR1_UE;                // включаем usart
 
    NVIC_EnableIRQ(USART1_IRQn);                // включаем обратно прерывания
} 

void TIM3CH3PWMInit () {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;         // включаем тактирование TIM3

    TIM3->ARR    = 255 - 1;                     // устанавливаем диапазон подсчета [0...255] 
    TIM3->CCER  |= TIM_CCER_CC3E;               // включаем 3 канал таймера
    TIM3->CCMR2 |= TIM_CCMR2_OC3M;              // конфигурируем режим выхода прямой ШИМ.
    TIM3->CR1   |= TIM_CR1_CEN;                 // включаем таймер
}

void TIM3CH3PWMConfigure (uint32_t freq, uint8_t rate) {
    if (freq == 0) {                            // обрабатываем случай когда частота 0
        rate = 0;
        freq = 1;
    }

    TIM3->PSC = (APBFreq / 256) / freq - 1;     // устанавлмваем прескейлер, для получения искомой частоты
    TIM3->CCR3 = 255 - rate;                    // устанавливаем скважность в третьем канале
}
