#include <stm32usart.h>

extern "C" void USART1_IRQHandler () {
    st32::USARTDevice::usart1.tick(USART1->ISR);
}
extern "C" void USART2_IRQHandler () {
    st32::USARTDevice::usart2.tick(USART2->ISR);
}

namespace st32 {

void enableUSART1 (const Pin& rx, const Pin& tx, uint32_t baudRate) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    USARTDevice::usart1.attachPinsToUSART(rx, tx);
    USARTDevice::usart1.setBaudRate(baudRate);
}

void enableUSART2 (const Pin& rx, const Pin& tx, uint32_t baudRate) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USARTDevice::usart2.attachPinsToUSART(rx, tx);
    USARTDevice::usart2.setBaudRate(baudRate);
}

USARTDevice USARTDevice::usart1 = USARTDevice(USART1, USART1_IRQn);
USARTDevice USARTDevice::usart2 = USARTDevice(USART2, USART2_IRQn);

USARTDevice::USARTDevice (USART_TypeDef* USARTx, IRQn_Type usartIRQ) : USARTx(USARTx), usartIRQ(usartIRQ) {}

void USARTDevice::attachPinsToUSART (const Pin& rx, const Pin& tx) {
    // configure
    
    this->tx = tx;
    this->rx = rx;

    if (tx != Pin::Unavailable) {
        Pin::AFType af = Pin::AFType::NO;

        if (tx == PA2)  af = Pin::AFType::AF1;
        if (tx == PA9)  af = Pin::AFType::AF1;
        if (tx == PA14) af = Pin::AFType::AF1;
        if (tx == PB6)  af = Pin::AFType::AF0;

        tx.configure(Pin::PinMode::ALTERNATE, Pin::PullType::NO, Pin::SpeedType::LOW, af);
    }

    if (rx != Pin::Unavailable) {
        Pin::AFType af = Pin::AFType::NO;

        if (tx == PA3)  af = Pin::AFType::AF1;
        if (tx == PA10) af = Pin::AFType::AF1;
        if (tx == PA15) af = Pin::AFType::AF1;
        if (tx == PB7)  af = Pin::AFType::AF0;

        tx.configure(Pin::PinMode::ALTERNATE, Pin::PullType::NO, Pin::SpeedType::LOW, af);
    }
}

void USARTDevice::setBaudRate (uint32_t baudRate) {
    NVIC_DisableIRQ(usartIRQ);
    USARTx->BRR   = APBClock / baudRate;

    if (tx != Pin::Unavailable) {
        USARTx->CR1 |= USART_CR1_TE;
        USARTx->CR1 |= USART_CR1_UE;
    }

    if (rx != Pin::Unavailable) {
        USARTx->CR1 |= USART_CR1_RXNEIE;
        USARTx->CR1 |= USART_CR1_RE;
        USARTx->CR1 |= USART_CR1_UE;
    }

    NVIC_EnableIRQ(usartIRQ);
}

void USARTDevice::tick (uint32_t flags) {
    auto& Led = st32::PC8;
    static uint8_t u = 1;

    Led.digitalWrite(u);
    u = !u;
    
    NVIC_DisableIRQ(usartIRQ);
    
    if (flags & USART_ISR_TXE) {
        if (outbuffer.size() == 0) {
            readyToTransmit = true;
        } else {
            switch (outbuffer.front().type) {
            case USARTDevice::USARTTask::Type::SINGLE:
                USARTx->TDR  = reinterpret_cast<const uint32_t>(outbuffer.front().begin); 
                USARTx->CR1 |= USART_CR1_TXEIE;

                outbuffer.pop();

                break;
            case USARTDevice::USARTTask::Type::REMOTE:
                USARTx->TDR  = *outbuffer.front().begin;
                USARTx->CR1 |= USART_CR1_TXEIE;

                outbuffer.front().begin++;

                if (outbuffer.front().begin == outbuffer.front().end) {
                    outbuffer.pop();
                }
                break;
            }
        }
    }
    
    if (flags & USART_ISR_RXNE) {
        inbuffer.push(USARTx->RDR);
        (void)USARTx->RDR;
        USARTx->CR1 |= USART_CR1_RXNEIE;
    }

    NVIC_EnableIRQ(usartIRQ);
}

void USARTDevice::send (uint8_t byte) {
    outbuffer.push({USARTDevice::USARTTask::Type::SINGLE, reinterpret_cast<uint8_t*>(byte), nullptr});
    if (readyToTransmit) {
        readyToTransmit = false;
        tick(USART_ISR_TXE);
    }
}

void USARTDevice::send (const uint8_t* begin, const uint8_t* end) {
    outbuffer.push({USARTDevice::USARTTask::Type::REMOTE, begin, end});
    if (readyToTransmit) {
        readyToTransmit = false;
        tick(USART_ISR_TXE);
    }
}

void USARTDevice::send (const void* begin, const void* end) {
    send(reinterpret_cast<const uint8_t*>(begin), reinterpret_cast<const uint8_t*>(end));
}

bool USARTDevice::transmitBufferEmpty () const {
    return outbuffer.size() == 0;
}

bool USARTDevice::reciveBufferEmpty () const {
    return inbuffer.size() == 0;
}

uint32_t USARTDevice::peek () const {
    if (inbuffer.size() == 0) return EOR;

    return inbuffer.front();
}

uint32_t USARTDevice::get () {
    if (inbuffer.size() == 0) return EOR;

    uint8_t res = inbuffer.front();
    inbuffer.pop();
    return res;
}

}