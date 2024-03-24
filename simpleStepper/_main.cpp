#include <stm32f0xx.h>
#include <stm32f051x8.h>
#include <system_stm32f0xx.h>

int main () {
    //SystemInit();
    /*
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN<<(pin>>26); установка пина
    RCC - глобальная пременная задающая нашу структуру

    GPIO:   IO input/output
                IDR - read only  - input data register
                ODR - write only - output data register
                BRR - write only - bit reset - быстрый сброс ODR (за одну команду)
            Af - альтернативные функция (по четыре бита на регистр)
                AFRH
                AFRL
            conf - конфиг
                PUPDR  - подтяжка
                OSPEED - позволяет менять
                OTYPER - выбор альтернативной функции (input/out/analog/alternative)
    
    !!! прервыания биндятся на номер пина, нельзя одновременно PA1 и PB1
    */
    // GPIOA->PUPDR |= GPIO_PUPDR_PUPDR11_1;
    // GPIO_PUPDR_PUPDR11_Msk;
    
    GPIOC->

    while (1) {
    /* USER CODE END WHILE */

        GPIOC->ODR ^= GPIO_ODR_8;


        for (uint32_t i = 0; i < 10000000; i++);
    /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
    // y = x^2 - sqrt(t)
    return 0;
}