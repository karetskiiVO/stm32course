#ifndef STM32USART_h
#define STM32USART_h

extern "C" void USART1_IRQHandler ();
extern "C" void USART2_IRQHandler ();

#include <stm32lib.h>
#include <iobuffer.h>

namespace st32 {
const uint32_t APBClock = 8'000'000;

void enableUSART1 (const Pin& rx, const Pin& tx, uint32_t baudRate);
void enableUSART2 (const Pin& rx, const Pin& tx, uint32_t baudRate);

// Empty recive
const uint32_t EOR = -1;

class USARTDevice {
    USART_TypeDef* USARTx;
    IRQn_Type      usartIRQ;

    Pin            rx = Pin::Unavailable;
    Pin            tx = Pin::Unavailable;

    bool readyToTransmit = true;

    struct USARTTask {
        enum class Type : uint8_t {
            SINGLE,
            REMOTE,
        };

        Type type;
        
        const uint8_t* begin;
        const uint8_t* end;
    };

    FIFObuffer<USARTTask, 32> outbuffer;
    FIFObuffer<uint8_t, 64> inbuffer;

    USARTDevice (USART_TypeDef* USARTx, IRQn_Type usartIRQ);

    void tick (uint32_t flags);
public:
    ~USARTDevice() = default;

    USARTDevice(USARTDevice&&)                  = delete;
    USARTDevice(const USARTDevice&)             = delete;
    USARTDevice& operator= (USARTDevice&&)      = delete;
    USARTDevice& operator= (const USARTDevice&) = delete;

    void setBaudRate (uint32_t baudRate);

    void attachPinsToUSART (const Pin& rx, const Pin& tx);

    void send (uint8_t byte);
    void send (const void* begin, const void* end); 
    void send (const uint8_t* begin, const uint8_t* end); 

    bool transmitBufferEmpty () const;
    bool reciveBufferEmpty () const;

    uint32_t peek () const;
    uint32_t get ();

    static USARTDevice usart1;
    static USARTDevice usart2;

    friend void ::USART1_IRQHandler ();
    friend void ::USART2_IRQHandler ();
};

}

#endif // stm32uart.h