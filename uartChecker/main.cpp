#include <stm32lib.h>
#include <stm32time.h>
#include <stm32usart.h>

// PB7 - usart1 - transmit
// PA2 - usart2 - recive 
int main (void) {
    st32::USARTDevice::usart1.setMode(st32::USARTDevice::USARTMode::TRANSMIT);
    // st32::USARTDevice::usart2.setMode(st32::USARTDevice::USARTMode::RECIVE);

    const uint8_t* msg = (const uint8_t*)"Around the world, around the world,\n" 
                         "Around the world, around the world,\n"  
                         "Around the world, around the world,\n"
                         "Around the world, around the world!\n";

    st32::USARTDevice::usart1.setMode(st32::USARTDevice::USARTMode::TRANSMIT_REMOTE);
    st32::USARTDevice::usart1.sendRemote(msg, msg + 144);
    while (true) {
    }

    return 0;
}