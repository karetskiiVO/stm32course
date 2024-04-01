#include <stm32lib.h>
#include <climits>

using namespace st32;

namespace {
    class StlibInit {
        void TIM6init () {
            RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // enable timer 6

            TIM6->PSC = 8 - 1;                  // 8MHz/8 = 1MHz
            TIM6->ARR = TIM_ARR_ARR;            // MAX arr value

            TIM6->CR1 |=  TIM_CR1_CEN;          // counter enable
            while (!(TIM6->SR & TIM_SR_UIF)) {} // update interupt flag
        }

        void GPIOinit () {
            RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN
                        |  RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
        }

        void SYSTICKinit () {
            static_assert((CoreFrequency % IntFrequency == 0) && "IntFrequency must be multiple");
            static_assert((CoreFrequency / IntFrequency <= (1 << 24)) && "IntFrequency is to low");

            // RCC->APB2ENR |= RCC_APB2ENR_SYSTICKEN;
            SysTick->VAL  = CoreFrequency / IntFrequency - 1;
            SysTick->LOAD = CoreFrequency / IntFrequency - 1;
            SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
        };
    public:
        StlibInit () {
            TIM6init();
            GPIOinit();
            SYSTICKinit();
        }
    };

    StlibInit init;
}

Pin::Pin (GPIO_TypeDef* GPIO, uint16_t num) : GPIO(GPIO), num(num) {}

void Pin::setDigitalOutput (PullType pull, SpeedType speed) const {
    /* Настраиваем режим работы порта */
    GPIO->MODER |= (uint16_t)1 << (num << (uint16_t)1);

    /* Настраиваем Output type в режим Push-Pull */
    GPIO->OTYPER &= ~((uint16_t)1 << num);

    /* устанавливаем подтяжку*/
    GPIO->PUPDR &= ~((uint16_t)0b11 << (num << (uint16_t)1));
    GPIO->PUPDR |= (uint16_t)static_cast<uint16_t>(pull) << (num << (uint16_t)1);

    /* Настраиваем скорость работы порта в spped */
    GPIO->OSPEEDR &= ~((uint16_t)0b11 << (num << (uint16_t)1));
    GPIO->OSPEEDR |= (uint16_t)static_cast<uint16_t>(speed) << (num << (uint16_t)1);
}

void Pin::digitalWrite (uint8_t bit) const {
    if (bit) {
        GPIO->ODR |=  ((uint16_t)1 << num);
    } else {
        GPIO->ODR &= ~((uint16_t)1 << num);
    }
}