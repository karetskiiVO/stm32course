#include <system.h>

namespace stm32 {

void SysTickInit () {
    const uint16_t chunk = 60000;
    SysTick->LOAD = chunk - 1;
    SysTick->VAL  = chunk - 1;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void delayms (uint32_t ms) {
    const uint16_t chunk = 60000;

    ms *= 1000 / 8;

    for (; ms > chunk; ms -= chunk) {
        SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
        SysTick->VAL = chunk;
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {}
    }

    SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
    SysTick->VAL = ms;
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {}
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

}