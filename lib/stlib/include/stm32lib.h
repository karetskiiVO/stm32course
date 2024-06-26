#pragma once
#ifndef STM32LIB_h
#define STM32LIB_h

#include <stm32core/stm32f0xx.h>
#include <stm32core/stm32f051x8.h>
#include <stm32core/system_stm32f0xx.h>

#include <map>
#include <cstring>

namespace st32 {

class Pin {
    GPIO_TypeDef* GPIO;
    uint16_t num;
    
public:
    enum class SpeedType : uint32_t {
        LOW    = 0b00,
        MEDUIM = 0b01,
        HIGH   = 0b11,
    };
    enum class PullType : uint32_t {
        NO       = 0b00,
        PULLUP   = 0b01,
        PULLDOWN = 0b11,
    };
    enum class PinMode : uint32_t {
        INPUT     = 0b00,
        OUTPUT    = 0b01,
        ALTERNATE = 0b10,
        ANALOG    = 0b11,
    };
    enum class AFType : uint32_t {
        NO   = 0,
        AF0  = 0,
        AF1  = 1,
        AF2  = 2,
        AF3  = 3,
        AF4  = 4,
        AF5  = 5,
        AF6  = 6,
        AF7  = 7,
        AF8  = 8,
        AF9  = 9,
        AF10 = 10,
        AF11 = 11,
        AF12 = 12,
        AF14 = 14,
        AF15 = 15,
    };

    Pin (GPIO_TypeDef* GPIO, uint16_t num);

    static const Pin Unavailable;

    void digitalWrite (uint8_t bit) const;
    void configure (PinMode mode, PullType pull, SpeedType speed, AFType af) const;

    bool operator== (const Pin& other) const;
    bool operator!= (const Pin& other) const;
};

#if true
const Pin PA0 {GPIOA,  0};
const Pin PA1 {GPIOA,  1};
const Pin PA2 {GPIOA,  2};
const Pin PA3 {GPIOA,  3};
const Pin PA4 {GPIOA,  4};
const Pin PA5 {GPIOA,  5};
const Pin PA6 {GPIOA,  6};
const Pin PA7 {GPIOA,  7};
const Pin PA8 {GPIOA,  8};
const Pin PA9 {GPIOA,  9};
const Pin PA10{GPIOA, 10};
const Pin PA11{GPIOA, 11};
const Pin PA12{GPIOA, 12};
const Pin PA13{GPIOA, 13};
const Pin PA14{GPIOA, 14};
const Pin PA15{GPIOA, 15};

const Pin PB0 {GPIOB,  0};
const Pin PB1 {GPIOB,  1};
const Pin PB2 {GPIOB,  2};
const Pin PB3 {GPIOB,  3};
const Pin PB4 {GPIOB,  4};
const Pin PB5 {GPIOB,  5};
const Pin PB6 {GPIOB,  6};
const Pin PB7 {GPIOB,  7};
const Pin PB8 {GPIOB,  8};
const Pin PB9 {GPIOB,  9};
const Pin PB10{GPIOB, 10};
const Pin PB11{GPIOB, 11};
const Pin PB12{GPIOB, 12};
const Pin PB13{GPIOB, 13};
const Pin PB14{GPIOB, 14};
const Pin PB15{GPIOB, 15};

const Pin PC0 {GPIOC,  0};
const Pin PC1 {GPIOC,  1};
const Pin PC2 {GPIOC,  2};
const Pin PC3 {GPIOC,  3};
const Pin PC4 {GPIOC,  4};
const Pin PC5 {GPIOC,  5};
const Pin PC6 {GPIOC,  6};
const Pin PC7 {GPIOC,  7};
const Pin PC8 {GPIOC,  8};
const Pin PC9 {GPIOC,  9};
const Pin PC10{GPIOC, 10};
const Pin PC11{GPIOC, 11};
const Pin PC12{GPIOC, 12};
const Pin PC13{GPIOC, 13};
const Pin PC14{GPIOC, 14};
const Pin PC15{GPIOC, 15};

const Pin PD0 {GPIOD,  0};
const Pin PD1 {GPIOD,  1};
const Pin PD2 {GPIOD,  2};
const Pin PD3 {GPIOD,  3};
const Pin PD4 {GPIOD,  4};
const Pin PD5 {GPIOD,  5};
const Pin PD6 {GPIOD,  6};
const Pin PD7 {GPIOD,  7};
const Pin PD8 {GPIOD,  8};
const Pin PD9 {GPIOD,  9};
const Pin PD10{GPIOD, 10};
const Pin PD11{GPIOD, 11};
const Pin PD12{GPIOD, 12};
const Pin PD13{GPIOD, 13};
const Pin PD14{GPIOD, 14};
const Pin PD15{GPIOD, 15};
#endif

}

#endif // stm32lib.h