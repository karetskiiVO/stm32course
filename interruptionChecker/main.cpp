#include <stm32lib.h>
#include <stm32stepper.h>

int main (void) {
    using namespace st32;

    Pin servoOut = PB6;
    servoOut.setDigitalOutput();

    while (1) {
        for (uint16_t i = 255 / 50; i < 2 * 255 / 50; i++) {
            servoOut.PWMWrite(i);
            timer::delay_ms(100);
        }
        for (uint16_t i = 2 * 255 / 50; i > 255 / 50; i--) {
            servoOut.PWMWrite(i);
            timer::delay_ms(200);
        }
    }
    
    return 0;
}