#include <system.h>

#include <usb.h>
#include <usb_hid.h>

const uint32_t Led = 6;

int main () {
    using namespace stm32;

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    configurePin(GPIOB, Led, PinMode::OUTPUT, SpeedMode::LOW, PullMode::NOPULL, 0);

    SysTickInit();

    USB_setup();

    // USB_ADDR0_TX_ADDR0_TX;
    // USB->DADDR;
    // USB_ADDR0

    while (1) {
        
        usb_class_poll();
        delayms(1);
    }

    return 0;
}
