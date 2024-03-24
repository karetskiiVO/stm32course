#pragma once
#ifndef STLIB_h
#define STLIB_h

#include <stm32f0xx.h>
#include <stm32f051x8.h>
#include <system_stm32f0xx.h>

namespace st {

class timer {
public:
    timer () = delete;

    timer(timer&)  = delete;
    timer(timer&&) = delete;

    static void delay_us(uint16_t us);
    static void delay_ms(uint16_t ms);
};

class Pin {
    GPIO_TypeDef* GPIO;
    uint16_t num;
public:
    Pin (GPIO_TypeDef* GPIO, uint16_t num);

    void writeBit (uint8_t bit) const;
    void setOMode () const;
};

class Motor {};
class Stepper : public Motor {
public:
    enum class Direction : uint8_t {
        CW  = 0x01,
        CCW = 0xff,
    };
private:
    Pin in1;
    Pin in2;
    Pin in3;
    Pin in4;

    #define STPS_CNT (uint8_t)8
    const uint8_t phases[STPS_CNT] = {0b1001, 0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000};

    uint8_t   currentPhase = 0;
    Direction currentDir;

    void writePhase (uint8_t phmsk);
public:
    Stepper (const Pin& in1, const Pin& in2, const Pin& in3, const Pin& in4, Direction dir = Direction::CW);

    void step ();
    void setDirection (Direction dir);
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

#endif