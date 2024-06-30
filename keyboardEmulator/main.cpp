#include <stm32l1xx.h>
#include <stm32l152xc.h>
#include <system_stm32l1xx.h>

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

const uint32_t Led = 6;

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

void USBInit () {
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->PMC  &= ~SYSCFG_PMC_USB_PU;

    USB->CNTR = USB_CNTR_FRES; // Force USB Reset
    
    delayms(8);
    USB->CNTR   = 0;
    USB->BTABLE = 0;
    USB->DADDR  = 0;
    USB->ISTR   = 0;
    USB->CNTR   = USB_CNTR_RESETM | USB_CNTR_WKUPM;

    NVIC_EnableIRQ(USB_LP_IRQn);

    SYSCFG->PMC |= SYSCFG_PMC_USB_PU;
}

extern "C" void USB_LP_IRQHandler (void) {
    GPIOB->ODR |= 1 << Led;

    if (USB->ISTR & USB_ISTR_CTR) {}
    if (USB->ISTR & USB_ISTR_SOF) {}
    if (USB->ISTR & USB_ISTR_RESET) {}
    if (USB->ISTR & USB_ISTR_SUSP) {} // suspend -> still no connection, may sleep
    if (USB->ISTR & USB_ISTR_WKUP) {} // wakeup
}

int main () {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    configurePin(GPIOB, Led, PinMode::OUTPUT, SpeedMode::LOW, PullMode::NOPULL, 0);

    SysTickInit();

    USBInit();
    __enable_irq();

    // USB_ADDR0_TX_ADDR0_TX;
    // USB->DADDR;
    // USB_ADDR0

    while (1) {}

    return 0;
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



