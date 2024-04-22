#ifndef STM32USART_h
#define STM32USART_h

extern "C" void USART1_IRQHandler ();
extern "C" void USART2_IRQHandler ();

#include <stm32lib.h>

namespace st32 {
const uint32_t APBClock = 8'000'000; // implement

#define IO_Buffer_Size 16

template <typename T, uint32_t Capacity>
class FIFObufer {
    uint32_t size_;
    uint32_t fst, lst;

    T storage[Capacity];
public:
    FIFObufer () : size_(0), fst(0), lst(0) {}

    void clear () {
        size_ = 0;
        fst   = 0;
        lst   = 0;
    }

    void push (T elem) {
        storage[lst] = elem;
        if (size_ == Capacity) {
            fst = (fst + 1) & Capacity;
        } else {
            size_++;
        }
        lst = (lst + 1) % Capacity;
    }

    T front () const {
        return storage[fst];
    }

    uint32_t size () const {
        return size_;
    }

    void pop () {
        if (size_ == 0) return;
        fst = (fst + 1) % Capacity;
        size_--;
    }
};

class USARTDevice {
public:
    enum class USARTMode {
        TRANSMIT,
        TRANSMIT_REMOTE,
        RECIVE,
        RECIVE_REMOTE,
    };
private:
    uint32_t       RCC_APB1ENR_USARTxEN;
    USART_TypeDef* USARTx;
    USARTMode      mode;
    uint32_t       baudRate;
    IRQn_Type      usartIRQ;
    Pin            attachedPin;

    FIFObufer<uint8_t, IO_Buffer_Size> ioBuffer;

    uint8_t* remoteBegin = nullptr;
    uint8_t* remoteEnd   = nullptr;

    USARTDevice (
        USART_TypeDef*     USARTx,
        volatile uint32_t& APB,
        uint32_t           RCC_APB1ENR_USARTxEN, 
        IRQn_Type          usartIRQ,
        uint32_t           baudRate,
        const Pin&         attachedPin
    );

    void tick ();
public:
    ~USARTDevice() = default;

    USARTDevice(USARTDevice&&)                  = delete;
    USARTDevice(const USARTDevice&)             = delete;
    USARTDevice& operator= (USARTDevice&&)      = delete;
    USARTDevice& operator= (const USARTDevice&) = delete;

    void setBaudRate (uint32_t baudRate); 
    void setMode (USARTMode mode);

    void send (uint8_t byte);
    void send (uint8_t* begin, uint8_t* end);

    void sendRemote (uint8_t* begin, uint8_t* end);

    static USARTDevice usart1;
    static USARTDevice usart2;

    friend void ::USART1_IRQHandler ();
    friend void ::USART2_IRQHandler ();
};

}

#endif // stm32uart.h