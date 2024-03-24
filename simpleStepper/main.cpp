#include <stlib.h>

int main (void) {
    using namespace st;

    Stepper st(PB3, PD2, PC12, PC11);

    while (1) {
        st.setDirection(Stepper::Direction::CW);

        for (uint16_t i = 0; i < (1 << 12); i++) {
            st.step();
            timer::delay_ms(1);
        }

        st.setDirection(Stepper::Direction::CCW);

        for (uint16_t i = 0; i < (1 << 12); i++) {
            st.step();
            timer::delay_ms(1);
        }
    }
}