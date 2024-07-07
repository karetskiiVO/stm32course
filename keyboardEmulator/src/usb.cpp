#include <usb.h>
#include <system.h>
namespace stm32 {

struct usb_epdata_t {
    volatile uint32_t usb_tx_addr;
    volatile union{
        uint32_t usb_tx_count; //SINGLE mode, TX count
        struct{                //DOUBLE mode, RX struct
            uint32_t tx_count:10;
            uint32_t tx_num_blocks:5;
            uint32_t tx_blocksize:1;
        };
    };
    volatile uint32_t usb_rx_addr;
    volatile union{
        uint32_t usb_rx_count; //DOUBLE mode, TX count
        struct{                //SINGLE mode, RX struct
            uint32_t rx_count:10;
            uint32_t rx_num_blocks:5;
            uint32_t rx_blocksize:1;
        };
    };
};

#define USBENDPOINTS (8)
#define usbEpdata ((volatile usb_ep))

__attribute__((weak)) void usb_class_init       () {}
__attribute__((weak)) void usb_class_disconnect () {}

void USBInit () {
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->PMC  &= ~SYSCFG_PMC_USB_PU;

    USB->CNTR = USB_CNTR_FRES; // Force USB Reset
    
    delayms(8);
    USB->CNTR   = 0;
    USB->ISTR   = 0;
    USB->CNTR   = USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;

    NVIC_EnableIRQ(USB_LP_IRQn);

    SYSCFG->PMC |= SYSCFG_PMC_USB_PU;
}


extern "C" void USB_LP_IRQHandler (void) {
    if (USB->ISTR & USB_ISTR_RESET) {
        usb_class_disconnect();
        #ifdef USBLIB_SOF_ENABLE
        USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SOFM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
        #else
        USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
        #endif
        // lastaddr = LASTADDR_DEFAULT;
        USB->DADDR = USB_DADDR_EF;
        // for(uint8_t i=0; i<STM32ENDPOINTS; i++)epfunc_in[i] = epfunc_out[i] = endp_callback_default;
        
        // state is default - wait for enumeration
        USB->ISTR = (uint16_t)~USB_ISTR_RESET;
        // usb_ep_init(0x00, USB_ENDP_CTRL, USB_EP0_BUFSZ, ep0_out);
        // usb_ep_init(0x80, USB_ENDP_CTRL, USB_EP0_BUFSZ, ep0_in);
        // ep0_buf = NULL;
        usb_class_init();
    }

    if (USB->ISTR & USB_ISTR_WKUP) { // wakeup
    }

    while (USB->ISTR & USB_ISTR_CTR) {
    }
}

}