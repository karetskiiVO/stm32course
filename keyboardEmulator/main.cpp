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


int main () {
    RCC->APB1ENR |= RCC_APB1ENR_USBEN; // включаем USB
    
    USB_ADDR0_TX_ADDR0_TX;
    USB->DADDR;
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



