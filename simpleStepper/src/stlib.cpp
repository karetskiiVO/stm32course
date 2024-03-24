#include <stlib.h>
#include <stm32f0xx.h>
#include <stm32f051x8.h>
#include <system_stm32f0xx.h>

#include <utility>

using namespace st;

namespace {
    class StlibInit {
    public:
        StlibInit () {
            RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // enable timer 6

            TIM6->PSC = 8 - 1;                  // 8MHz/8 = 1MHz
            TIM6->ARR = TIM_ARR_ARR;            // MAX arr value

            TIM6->CR1 |=  TIM_CR1_CEN;          // counter enable
            while (!(TIM6->SR & TIM_SR_UIF)) {} // update interupt flag

            RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN
                        |  RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
        }
    };

    StlibInit init;
}

void timer::delay_us(uint16_t us) {
    TIM6->CNT = 0;
    while (TIM6->CNT < us);
}
void timer::delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) delay_us(1000);
}

Pin::Pin (GPIO_TypeDef* GPIO, uint16_t num) : GPIO(GPIO), num(num) {}

void Pin::setOMode () const {

    /* Настраиваем режим работы порта */
    GPIO->MODER |= (uint16_t)1 << (num << (uint16_t)1);

    // rewrite
    /* Настраиваем Output type в режим Push-Pull */
    GPIO->OTYPER &= ~((uint16_t)1 << num);

    /* Настраиваем скорость работы порта в Fast */
    GPIO->OSPEEDR |= (uint16_t)0b11 << (num << (uint16_t)1);
}

void Pin::writeBit (uint8_t bit) const {
    if (bit) {
        GPIO->ODR |=  ((uint16_t)1 << num);
    } else {
        GPIO->ODR &= ~((uint16_t)1 << num);
    }
}

Stepper::Stepper (const Pin& in1, const Pin& in2, const Pin& in3, const Pin& in4, Direction dir)
    : in1(in1), in2(in2), in3(in3), in4(in4), currentPhase(0), currentDir(dir) {    
    in1.setOMode();
    in2.setOMode();
    in3.setOMode();
    in4.setOMode();
}

void Stepper::writePhase (uint8_t phmsk) {
    in1.writeBit((phmsk >> (uint8_t)0) & 1);
    in2.writeBit((phmsk >> (uint8_t)1) & 1);
    in3.writeBit((phmsk >> (uint8_t)2) & 1);
    in4.writeBit((phmsk >> (uint8_t)3) & 1);
}

void Stepper::step () {
    currentPhase = (currentPhase + static_cast<uint8_t>(currentDir)) % STPS_CNT;
    writePhase(phases[currentPhase]);
}

void Stepper::setDirection (Direction dir) {
    currentDir = dir;
}