#ifndef SYSTEM_h
#define SYSTEM_h

#include <stm32l1xx.h>
#include <stm32l152xc.h>
#include <system_stm32l1xx.h>

namespace stm32 {

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
void SysTickInit ();
void delayms (uint32_t ms);

}

#endif