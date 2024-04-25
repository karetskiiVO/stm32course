#include <climits>
#include <stm32lib.h>
#include <stm32time.h>

namespace st32 {

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

const Pin Pin::Unavailable = Pin{nullptr, UINT16_MAX};

void Pin::configure (PinMode mode, PullType pull, SpeedType speed, AFType af) const {
    /* Настраиваем режим работы порта */
    GPIO->MODER &= ~(uint32_t(0b11) << (num << uint32_t(1)));
    GPIO->MODER |= uint32_t(mode) << (num << uint32_t(1));

    /* Настраиваем Output type в режим Push-Pull */
    GPIO->OTYPER &= ~(uint32_t(1) << num);

    /* устанавливаем подтяжку*/
    GPIO->PUPDR &= ~(uint32_t(0b11) << (num << uint32_t(1)));
    GPIO->PUPDR |= uint32_t(pull) << (num << uint32_t(1));

    /* Настраиваем скорость работы порта в spped */
    GPIO->OSPEEDR &= ~(uint32_t(0b11) << (num << uint32_t(1)));
    GPIO->OSPEEDR |= uint32_t(speed) << (num << uint32_t(1));

    /* устанавливаем альтернативную функцию */
    GPIO->AFR[num / 8] &= ~(uint32_t(0x0F) << ((num % uint32_t(8)) * uint32_t(4)));
    GPIO->AFR[num / 8] |= uint32_t(af) << ((num % uint32_t(8)) * uint32_t(4));
}

void Pin::digitalWrite (uint8_t bit) const {
    if (bit != 0) {
        GPIO->BSRR = uint32_t(1) <<  uint32_t(num);
    } else {
        GPIO->BSRR = uint32_t(1) <<  uint32_t(num) << uint32_t(16);
    }
}

bool Pin::operator== (const Pin& other) const {
    return GPIO == other.GPIO && num == other.num;
}

bool Pin::operator!= (const Pin& other) const {
    return !(*this == other);
}

}