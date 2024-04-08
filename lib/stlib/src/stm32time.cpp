#include <stm32pwm.h>
#include <stm32time.h>

using namespace st32;

uint32_t timer::milliseconds    = 0;
uint32_t timer::submilliseconds = 0;

extern "C" void SysTick_Handler (void) {
    timer::submilliseconds++;

    if (timer::submilliseconds < IntFrequency / 1000) {
        timer::milliseconds++;
        timer::submilliseconds -= IntFrequency / 1000;
    }
}

void timer::delay_us(uint16_t us) {
    TIM6->CNT = 0;
    while (TIM6->CNT < us);
}
void timer::delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) delay_us(1000);
}

uint32_t timer::millis () {
    return milliseconds;
}

SoftwareTIM2PWMdevice* SoftwareTIM2PWMdevice::exemplar = nullptr;
