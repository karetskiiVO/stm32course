#include <stm32l1xx.h>
#include <stm32l152xc.h>
#include <system_stm32l1xx.h>

#include <system.h>
#include <usb.h>

const uint32_t Led = 6;

int main () {
    using namespace stm32;

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    configurePin(GPIOB, Led, PinMode::OUTPUT, SpeedMode::LOW, PullMode::NOPULL, 0);

    SysTickInit();

    USBInit();

    // USB_ADDR0_TX_ADDR0_TX;
    // USB->DADDR;
    // USB_ADDR0

    while (1) {}

    return 0;
}
