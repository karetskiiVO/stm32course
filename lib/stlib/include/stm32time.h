#ifndef STM32TIME_h
#define STM32TIME_h

#include <stm32lib.h>

extern "C" void SysTick_Handler (void); 

namespace st32 {
    
const uint32_t CoreFrequency = 8'000'000 / 8;
const uint32_t IntFrequency  = 10'000;

class timer {
    static uint32_t milliseconds;
    static uint32_t submilliseconds;

    //static void 

public:
    timer () = delete;

    timer(timer&)  = delete;
    timer(timer&&) = delete;

    static void delay_us(uint16_t us);
    static void delay_ms(uint16_t ms);

    static uint32_t millis ();

    friend void ::SysTick_Handler (void);
};

}

#endif // stm32time.h