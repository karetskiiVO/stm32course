#include <stm32lib.h>
#include <stm32time.h>
#include <stm32usart.h>

// PB7 - usart1 - transmit
// PA2 - usart2 - recive 
int main (void) {
    st32::USARTDevice::usart1.setMode(st32::USARTDevice::USARTMode::TRANSMIT);
    // st32::USARTDevice::usart2.setMode(st32::USARTDevice::USARTMode::RECIVE);

    for (uint8_t ch = 'a'; ch < 'a' + 4; ch++) {
        st32::USARTDevice::usart1.send(ch);
    }
    while (true) {
    }

    return 0;
}