#include <stm32lib.h>
#include <stm32time.h>
#include <stm32usart.h>

// PB7 - usart1 - transmit
// PA2 - usart2 - recive 
int main (void) {
    st32::enableUSART1(st32::PB6, st32::Pin::Unavailable, 9600);
    // st32::USARTDevice::usart2.setMode(st32::USARTDevice::USARTMode::RECIVE);

    const uint8_t* msg = (const uint8_t*)"Around the world, around the world,\n" 
                         "Around the world, around the world,\n"  
                         "Around the world, around the world,\n"
                         "Around the world, around the world!\n";

    for (uint8_t i = 0; i < 10; i++) st32::USARTDevice::usart1.send('=');
    st32::USARTDevice::usart1.send('\n');
    st32::USARTDevice::usart1.send(msg, msg + 144);
    while (true) {}

    return 0;
}