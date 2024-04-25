#include <stm32lib.h>
#include <stm32time.h>
#include <stm32usart.h>

#include <string>

// PB7 - usart1 - transmit
// PA2 - usart2 - recive

std::string getStr () {
    std::string res;
    while (true) {
        if (st32::USARTDevice::usart1.peek() == st32::EOR) continue;

        uint8_t newch = st32::USARTDevice::usart1.get();
        if (newch == ' ' || newch == '\n') break;
        if (newch == '\0') {
            if (res.length() != 0) {
                break;
            } else {
                continue;
            }
        }

        res.push_back(newch);
    }

    return res;
}

auto Led = st32::PC8;

int main (void) {
    Led.configure(st32::Pin::PinMode::OUTPUT, st32::Pin::PullType::NO, st32::Pin::SpeedType::LOW, st32::Pin::AFType::NO);
    st32::enableUSART1(st32::PB7, st32::PB6, 9600);


    while (true) {
        std::string buf = getStr();

        
        st32::USARTDevice::usart1.send(buf.c_str(), buf.c_str() + buf.length());
        st32::USARTDevice::usart1.send('\n');
        while (!st32::USARTDevice::usart1.transmitBufferEmpty()) {}
    }

    return 0;
}