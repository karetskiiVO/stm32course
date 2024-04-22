#define STM32PWM_h
#ifndef STM32PWM_h

#include <stm32lib.h>
#include <stm32time.h>

namespace st32 {

class PWMdevice {
public:
    virtual void setFreq   (float f)                      = 0;
    virtual void attachPin (const Pin& pin, uint32_t val) = 0; 
};

class SoftwareTIM2PWMdevice : public PWMdevice {
    static uint32_t BaseFreq;
    static uint16_t PWMresolution;

    static SoftwareTIM2PWMdevice* exemplar;
public:
    SoftwareTIM2PWMdevice (uint32_t freq, uint16_t resolution) {
        if (exemplar != nullptr) return;

        BaseFreq = freq;
        PWMresolution = resolution;
        exemplar = this;
        
        // setup counter mode
        TIM2->CR1  = (0b0 << TIM_CR1_ARPE_Pos) | (0b00 << TIM_CR1_CMS_Pos) | (0b0 << TIM_CR1_DIR_Pos) | (TIM_CR1_CEN); 
        
        // setup interruption mode
        TIM2->ARR  = PWMresolution;
    }

    void setFreq (float f) {
        
    }
    
    void attachPin (const Pin& pin, uint32_t val) {

    }

    #undef TIM
};

SoftwareTIM2PWMdevice tim1PWM{8'000'000, 1024};

}

// extern "C" void TIM1_

#endif // stm32pwm.h