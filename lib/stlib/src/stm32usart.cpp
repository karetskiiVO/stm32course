#include <stm32usart.h>

extern "C" void USART1_IRQHandler () {
    st32::USARTDevice::usart1.tick();
}
extern "C" void USART2_IRQHandler () {
    st32::USARTDevice::usart2.tick();
}

namespace st32 {

// остаются вопросы относительно шины
USARTDevice USARTDevice::usart1 = USARTDevice(
    USART1, RCC->APB2ENR, RCC_APB2ENR_USART1EN, IRQn_Type::USART1_IRQn, 9600, PB6
);
USARTDevice USARTDevice::usart2 = USARTDevice(
    USART2, RCC->APB1ENR, RCC_APB1ENR_USART2EN, IRQn_Type::USART2_IRQn, 9600, PA2
);

// прикрепить к пинам
USARTDevice::USARTDevice (
    USART_TypeDef*     USARTx,
    volatile uint32_t& APB,
    uint32_t           RCC_APB1ENR_USARTxEN, 
    IRQn_Type          usartIRQ,
    uint32_t           baudRate,
    const st32::Pin&   attachedPin
) : RCC_APB1ENR_USARTxEN(RCC_APB1ENR_USARTxEN), 
    USARTx(USARTx), 
    baudRate(baudRate),
    usartIRQ(usartIRQ),
    attachedPin(attachedPin) {
    APB |= RCC_APB1ENR_USARTxEN;
    mode = USARTMode::TRANSMIT;
    setBaudRate(baudRate); 
}

void USARTDevice::setBaudRate (uint32_t baudRate) {
    NVIC_DisableIRQ(usartIRQ);
    USARTx->BRR   = APBClock / baudRate;
    switch (mode) {
    case USARTMode::TRANSMIT:
    case USARTMode::TRANSMIT_REMOTE:
        attachedPin.configure(Pin::PinMode::ALTERNATE, Pin::PullType::NO, Pin::SpeedType::LOW, Pin::AFType::AF0);

        USARTx->CR1 |= USART_CR1_TE;  
        USARTx->CR1 |= USART_CR1_UE; 
        break;
    case USARTMode::RECIVE:
    case USARTMode::RECIVE_REMOTE:
        attachedPin.configure(Pin::PinMode::INPUT, Pin::PullType::NO, Pin::SpeedType::LOW, Pin::AFType::AF0);

        USARTx->CR1 |= USART_CR1_RXNEIE;
        USARTx->CR1 |= USART_CR1_RE;  
        USARTx->CR1 |= USART_CR1_UE; 
        break;
    }
    NVIC_EnableIRQ(usartIRQ);
} 

void USARTDevice::setMode (USARTMode mode) {
    if (this->mode == mode) return;

    this->mode = mode;
    setBaudRate(baudRate);
}

void USARTDevice::tick () {
    switch (mode) {
    case USARTMode::RECIVE:
        // implement
        break;
    case USARTMode::TRANSMIT:
        while (!(USARTx->ISR & USART_ISR_TXE)); 
        
        if (ioBuffer.size() == 0) {
            readyToWork = true;
            return;
        }

        USARTx->TDR  = ioBuffer.front(); 
        USARTx->CR1 |= USART_CR1_TXEIE;
        ioBuffer.pop();
        
        break;
    case USARTMode::RECIVE_REMOTE:
        // implement
        break;
    case USARTMode::TRANSMIT_REMOTE:
        while(!(USARTx->ISR & USART_ISR_TXE)); 
        
        if (remoteBegin == remoteEnd) {
            readyToWork = true;
            return;
        }

        USARTx->TDR  = *remoteBegin;
        USARTx->CR1 |= USART_CR1_TXEIE;
        remoteBegin++;

        break;
    }
}

void USARTDevice::send (uint8_t byte) {
    if (mode != USARTMode::TRANSMIT) return;
    ioBuffer.push(byte);

    if (readyToWork) {
        readyToWork = false;
        tick();
    }
}

void USARTDevice::send (const uint8_t* begin, const uint8_t* end) {
    if (mode != USARTMode::TRANSMIT) return;
    for (; begin < end; begin++) {
        ioBuffer.push(*begin);
    }

    if (readyToWork) {
        readyToWork = false;
        tick();
    }
}

void USARTDevice::sendRemote (const uint8_t* begin, const uint8_t* end) {
    if (mode != USARTMode::TRANSMIT_REMOTE) return;
    // create remote queue

    remoteBegin = begin;
    remoteEnd   = end;

    if (readyToWork) {
        readyToWork = false;
        tick();
    }
}

}