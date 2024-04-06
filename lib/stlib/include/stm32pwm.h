#ifndef STM32PWM_h
#define STM32PWM_h

#include <cmath>
#include <stm32lib.h>

#include <vector>
#include <algorithm>

namespace st32 {

class SoftwarePWM {
public:
    // clock frequency in Hz
    const uint32_t BaseFreq = 8'000'000;
private:
    struct PWMtask {
        Pin p;
        uint32_t val;
    };

    float currFreq;

    static void init () {
        // setup counter mode
        TIM1->CR1  = (0b0 << TIM_CR1_ARPE) | (0b00 << TIM_CR1_CMS_Pos) | (0b0 << TIM_CR1_DIR_Pos) | (TIM_CR1_CEN); 
        
        // setup interruption mode
        TIM1->ARR  = UINT16_MAX;
    }

    static std::vector<PWMtask> newPins;
    static std::vector<PWMtask> tasks;
public:
    SoftwarePWM  () = delete;

    void setPwmFrequency (float f) {
        uint16_t psc = std::round(BaseFreq / f);
        currFreq = BaseFreq * 1.0 / psc;

        TIM1->PSC = psc;
    }
};

}

#endif // stm32pwm.h