#include <stm32lib.h>
#include <stm32stepper.h>

int main (void) {
    using namespace st32;

    struct{uint32_t await, measured;} results[100];


    for (uint32_t i = 0; i < 100; i++) {
        results[i].await = (i + 1) * 10;
        uint32_t start = timer::millis();
        timer::delay_ms(results[i].await);
        uint32_t finish = timer::millis();

        results[i].measured = finish - start;
    }

    return 0;
}