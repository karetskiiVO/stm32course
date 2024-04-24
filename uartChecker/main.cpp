#include <stm32lib.h>
#include <stm32time.h>
#include <stm32usart.h>

// PB7 - usart1 - transmit
// PA2 - usart2 - recive 
int main (void) {
    st32::enableUSART1(st32::Pin::Unavailable, st32::PB6, 9600);

    const uint8_t* msg = (const uint8_t*)"1234";
    
    
    for (uint8_t ch = 0; ch < 10; ch++) st32::USARTDevice::usart1.send('=');
    st32::USARTDevice::usart1.send('\n');
    st32::USARTDevice::usart1.send(msg, msg + 4);
    while (true) {
    }

    return 0;
}