#include <stm32lib.h>
#include <stm32time.h>
#include <stm32usart.h>

#include <string>

// PB7 - usart1 - transmit
// PA2 - usart2 - recive
auto Led = st32::PC8;

std::string getStr () {
    std::string res;
    static uint8_t cnt = 0;

    if (cnt > 0) {
        Led.digitalWrite(1);
        st32::USARTDevice::usart1.send('A');
    }
    cnt++;

    while (true) {
        if (st32::USARTDevice::usart1.peek() == st32::EOR) {
            if (res.length() != 0) break;
            st32::timer::delay_ms(30);
            continue;
        }

        uint8_t newch = st32::USARTDevice::usart1.get(); 
        res.push_back(newch);
        st32::timer::delay_ms(30);
    }

    Led.digitalWrite(0);

    return res;
}

int main (void) {
    Led.configure(st32::Pin::PinMode::OUTPUT, st32::Pin::PullType::NO, st32::Pin::SpeedType::LOW, st32::Pin::AFType::NO);
    st32::enableUSART1(st32::PB7, st32::PB6, 9600);

    // std::string buf = getStr();
    // st32::USARTDevice::usart1.send(buf.c_str(), buf.c_str() + buf.length());
    st32::USARTDevice::usart1.send('A');
    uint32_t start = st32::timer::millis();
    while (st32::timer::millis() - start < 10);
    
    st32::USARTDevice::usart1.send('B');

    while (true) {}
    
    return 0;
}