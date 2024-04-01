#include <stm32lib.h>
#include <stm32stepper.h>

void systic_delay_ms (uint32_t ms) {
    SysTick->VAL  = 0;
    SysTick->LOAD = 8 * ms; // us << 3
    SysTick->CTRL = 0x1ul;

    for (uint32_t i = 0; i < 1000 * ms; i++)
        while ((SysTick->CTRL >> 16) & 1);
}

int main (void) {
    using namespace st32;

    Stepper st(PB3, PD2, PC12, PC11);

    while (1) {
        st.setDirection(Stepper::Direction::CW);

        for (uint16_t i = 0; i < (1 << 12); i++) {
            st.step();
            systic_delay_ms(1);
            //timer::delay_ms(1);
        }

        continue;

        st.setDirection(Stepper::Direction::CCW);

        for (uint16_t i = 0; i < (1 << 12); i++) {
            st.step();
            timer::delay_ms(1);
        }
    }
}