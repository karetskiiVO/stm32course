#include <usb.h>
#include <system.h>

namespace stm32 {

#define ZLPP ((void*)1)

//спасибо ST за наркоманскую работу с регистром!
#define ENDP_STAT_RX(num, stat) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ stat; }while(0)
#define ENDP_STAT_TX(num, stat) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ stat; }while(0)
#define ENDP_DTOG_RX(num, dtog) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ dtog; }while(0)
#define ENDP_DTOG_TX(num, dtog) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EPRX_STAT | USB_EPTX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX) ^ dtog; }while(0)
#define ENDP_CTR_RX_CLR(num) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT | USB_EP_CTR_RX)) | USB_EP_CTR_TX); }while(0)
#define ENDP_CTR_TX_CLR(num) do{USB_EPx(num) = ((USB_EPx(num) & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPRX_STAT | USB_EPTX_STAT | USB_EP_CTR_TX)) | USB_EP_CTR_RX); }while(0)

void usb_ep_write (uint8_t epnum, const uint16_t* buf, uint16_t size) {
    _usb_ep_write((epnum & 0x0F) * 2, buf, size);
}
void usb_ep_write_double (uint8_t epnum, const uint16_t* buf, uint16_t size) {
    epnum &= 0x0F;
    uint8_t idx = 2 * epnum + !!(USB_EPx(epnum) & USB_EP_DTOG_RX);
    ENDP_TOG(epnum, USB_EP_DTOG_RX);
    _usb_ep_write(idx, buf, size);
}

int usb_ep_read (uint8_t epnum, uint16_t* buf) {
    return _usb_ep_read((epnum & 0x0F)*2 + 1, buf);
}
int usb_ep_read_double (uint8_t epnum, uint16_t *buf) {
    uint8_t idx = !(USB_EPx(epnum) & USB_EP_DTOG_RX);
    int res = _usb_ep_read((epnum & 0x0F)*2 + idx, buf);
    ENDP_TOG((epnum & 0x0F), USB_EP_DTOG_TX);
    return res;
}

#define STM32ENDPOINTS (8)
#define usb_epdata   ((volatile usb_epdata_t*)(USB_PMAADDR))
#define LASTADDR_DEFAULT                (STM32ENDPOINTS * 8)

__attribute__((weak))void usb_class_init       () {}
__attribute__((weak))void usb_class_disconnect () {}
__attribute__((weak))void usb_class_poll       () {}
__attribute__((weak))void usb_class_sof        () {}
__attribute__((weak))char usb_class_ep0_in     (config_pack_t *req, void **data, uint16_t *size)       { return 0; }
__attribute__((weak))char usb_class_ep0_out    (config_pack_t *req, uint16_t offset, uint16_t rx_size) { return 0; }

static void endp_callback_default (uint8_t) {}

struct usb_epdata_t {
    volatile uint32_t usb_tx_addr;
    volatile union {
        uint32_t usb_tx_count; //SINGLE mode, TX count
        struct {               //DOUBLE mode, RX struct
            uint32_t tx_count      : 10;
            uint32_t tx_num_blocks :  5;
            uint32_t tx_blocksize  :  1;
        };
    };
    volatile uint32_t usb_rx_addr;
    volatile union {
        uint32_t usb_rx_count; //DOUBLE mode, TX count
        struct {               //SINGLE mode, RX struct
            uint32_t rx_count      : 10;
            uint32_t rx_num_blocks :  5;
            uint32_t rx_blocksize  :  1;
        };
    };
};

epfunc_t epfunc_in [STM32ENDPOINTS];
epfunc_t epfunc_out[STM32ENDPOINTS];

static config_pack_t setup_packet;

void USB_setup () {
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->PMC &=~ SYSCFG_PMC_USB_PU;

    USB->CNTR = USB_CNTR_FRES; 

    for(uint8_t i = 0; i<STM32ENDPOINTS; i++) {
        epfunc_in[i]  = endp_callback_default;
        epfunc_out[i] = endp_callback_default;
    }

    delayms(8);
    USB->CNTR   = 0;
    USB->BTABLE = 0;
    USB->DADDR  = 0;
    USB->ISTR   = 0;
    USB->CNTR   = USB_CNTR_RESETM | USB_CNTR_WKUPM;
    NVIC_EnableIRQ(USB_LP_IRQn);
    SYSCFG->PMC |= SYSCFG_PMC_USB_PU;
}

static const uint8_t* ep0_buf = nullptr;
static uint16_t ep0_count = 0;

static void ep0_in (uint8_t epnum) {
    if (ep0_buf == nullptr) return;
    uint16_t left = ep0_count;
    if (left > USB_EP0_BUFSZ) left = USB_EP0_BUFSZ;
    usb_ep_write(0, (uint16_t*)ep0_buf, left);
    
    ep0_count -= left;
    ep0_buf += left;
    
    if (left < USB_EP0_BUFSZ) ep0_buf = nullptr;
}

inline static void ep0_send (const uint16_t *buf, uint16_t size) {
    ep0_count = size;
    ep0_buf = (uint8_t*)buf;
    ep0_in(0x80);
}

static uint8_t configuration = 0;

static void ep0_out (uint8_t epnum) {
    static uint16_t bytesread = 0;
    uint16_t rxcount = usb_epdata[0].rx_count;
    uint8_t setup = !!(USB_EPx(0) & USB_EP_SETUP);

    if (setup) {
        if (setup_packet.bmRequestType & 0x80) { //предыдущий пакет=IN, этот точно будет запросом
            usb_ep_read(0, (uint16_t*)&setup_packet);
            bytesread = 0;
            rxcount = 0;
        } else { //предыдущий пакет=OUT, этот будет либо данными к нему, либо новым запросом
            if (bytesread >= setup_packet.wLength) {
                usb_ep_read(0, (uint16_t*)&setup_packet);
                bytesread = 0;
                rxcount = 0;
            }
        }
    }

    uint8_t req = setup_packet.bmRequestType & 0x7F;
  
    if (setup_packet.bmRequestType & 0x80) {
        //---IN---
        if (!setup) return; //какой-то костыль. Иначе вызывается два раза
        if (req == (USB_REQ_STANDARD | USB_REQ_DEVICE)) {
        switch (setup_packet.bRequest) {
            case GET_DESCRIPTOR: {
                    const void *data = ZLPP;
                    uint16_t size = 0;
                    usb_class_get_std_descr(setup_packet.wValue, &data, &size);
                    if (setup_packet.wLength < size) size = setup_packet.wLength;
                    ep0_send((const uint16_t*)data, size);
                    return;
                }
            case GET_STATUS:
                usb_ep_write(0, (uint16_t*)"\0", 2); // send status: Bus Powered
                return;
            case GET_CONFIGURATION:
                usb_ep_write(0, (uint16_t*)&configuration, 1);
                return;
            }
        }
        void *data = ZLPP;
        uint16_t size = 0;
        usb_class_ep0_in(&setup_packet, &data, &size);
        if (setup_packet.wLength < size) size = setup_packet.wLength;
        ep0_send((const uint16_t*)data, size);
    } else {
        //---OUT---
        if (req == (USB_REQ_STANDARD | USB_REQ_DEVICE)) {
            if (setup_packet.bRequest == SET_ADDRESS) {
                uint8_t USB_Addr = setup_packet.wValue;
                usb_ep_write(0, nullptr, 0);
                while ((USB_EPx(0) & USB_EPTX_STAT) == USB_EP_TX_VALID);
                USB->DADDR = USB_DADDR_EF | USB_Addr;
                return;
            } else if (setup_packet.bRequest == SET_CONFIGURATION) {
                configuration = setup_packet.wValue;
                usb_ep_write(0, nullptr, 0);
                return;
            }
        }
        if (!usb_class_ep0_out(&setup_packet, bytesread, rxcount)) {
            ENDP_STAT_RX(0, USB_EP_RX_VALID);
        }
        bytesread += rxcount;
        if (bytesread >= setup_packet.wLength) usb_ep_write(0, nullptr, 0);
    }
}

static uint16_t lastaddr = LASTADDR_DEFAULT;
void usb_ep_init (uint8_t epnum, uint8_t ep_type, uint16_t size, epfunc_t func) {
    if (func == nullptr) func = endp_callback_default;
    uint8_t dir_in = (epnum & 0x80);
    epnum &= 0x0F;
    

    ENDP_STAT_TX(epnum, dir_in ? USB_EP_TX_DIS : USB_EP_RX_DIS);
    
    uint16_t buf = USB_EPx(epnum);
    buf = (buf & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
    buf = (buf & ~USB_EPADDR_FIELD) | epnum;
    //buf = (buf & ~USB_EP_T_FIELD) | ep_type;
    buf &= ~USB_EP_T_FIELD;
    switch (ep_type) {
        case USB_ENDP_CTRL: 
            buf |= USB_EP_CONTROL;
            break;
        case USB_ENDP_BULK:
            buf |= USB_EP_BULK;
            break;
        case USB_ENDP_INTR:
            buf |= USB_EP_INTERRUPT;
            break;
        default: 
            buf |= USB_EP_ISOCHRONOUS; //в дескрипторах изохронные точки могут иметь расширенные настройки
    }
    USB_EPx(epnum) = buf;
    
    if (dir_in) {
        usb_epdata[epnum].usb_tx_addr = lastaddr;
        epfunc_in[epnum] = func;

        ENDP_STAT_TX(epnum, (ep_type & 0x03) == USB_ENDP_ISO ? USB_EP_TX_VALID : USB_EP_TX_NAK);
    } else {
        usb_epdata[epnum].usb_rx_addr = lastaddr;
        if (size < 64) {
            usb_epdata[epnum].rx_blocksize = 0;
            usb_epdata[epnum].rx_num_blocks = size / 2;
        } else {
            usb_epdata[epnum].rx_blocksize = 1;
            if (size < 32) size = 32;
            usb_epdata[epnum].rx_num_blocks = size / 32 - 1;
        }
        epfunc_out[epnum] = func;
        ENDP_STAT_RX(epnum, USB_EP_RX_VALID);
    }
    lastaddr += size;
}

void usb_ep_init_double(uint8_t epnum, uint8_t ep_type, uint16_t size, epfunc_t func ){
    if(func == nullptr) func = endp_callback_default;
    uint8_t dir_in = (epnum & 0x80);
    epnum &= 0x0F;
    
    ENDP_STAT_TX(epnum, USB_EP_TX_DIS);
    ENDP_STAT_RX(epnum, USB_EP_RX_DIS);
    
    uint16_t buf = USB_EPx(epnum);
    buf = (buf & ~(USB_EP_DTOG_RX | USB_EP_DTOG_TX | USB_EPTX_STAT | USB_EPRX_STAT)) | USB_EP_CTR_RX | USB_EP_CTR_TX;
    buf = (buf & ~USB_EPADDR_FIELD) | epnum;
    
    buf &=~ USB_EP_T_FIELD;
    switch (ep_type) {
        case USB_ENDP_CTRL: 
            buf |= USB_EP_CONTROL;
            break;
        case USB_ENDP_BULK: 
            buf |= USB_EP_BULK | USB_EP_KIND;
            break;
        case USB_ENDP_INTR:
            buf |= USB_EP_INTERRUPT;
            break;
        default: 
            buf |= USB_EP_ISOCHRONOUS; //в дескрипторах изохронные точки могут иметь расширенные настройки
    }
    USB_EPx(epnum) = buf;
  
#define USB_EP_SWBUF_TX     USB_EP_DTOG_RX
#define USB_EP_SWBUF_RX     USB_EP_DTOG_TX
    if (dir_in) {
        usb_epdata[epnum].usb_tx_addr = lastaddr;
        usb_epdata[epnum].usb_tx_count = 0;
        usb_epdata[epnum].usb_rx_addr = lastaddr + size;
        usb_epdata[epnum].usb_rx_count = 0;
        
        buf = USB_EPx(epnum);
        USB_EPx(epnum) = (buf ^ USB_EP_TX_VALID) & (USB_EPREG_MASK | USB_EPTX_STAT | USB_EP_DTOG_TX | USB_EP_SWBUF_TX);
    } else {
        usb_epdata[epnum].usb_rx_addr = lastaddr;
        usb_epdata[epnum].usb_tx_addr = lastaddr + size;
        if(size < 64){
            usb_epdata[epnum].rx_blocksize = usb_epdata[epnum].tx_blocksize = 0;
            usb_epdata[epnum].rx_num_blocks = usb_epdata[epnum].tx_num_blocks = size / 2;
        }else{
            usb_epdata[epnum].rx_blocksize = usb_epdata[epnum].tx_blocksize = 1;
            if (size < 32) size = 32;
            usb_epdata[epnum].rx_num_blocks = usb_epdata[epnum].tx_num_blocks = size / 32 - 1;
        }
        ENDP_STAT_RX(epnum, USB_EP_RX_VALID);
        ENDP_STAT_TX(epnum, USB_EP_TX_VALID);
    }
    epfunc_in[epnum] = func;
    epfunc_out[epnum]= func;
        
    lastaddr += 2*size;
}

}
/*
void USB_LP_IRQHandler (void) {
    using namespace stm32;
    // using stm32::usb_class_init;
    // using stm32::usb_class_disconnect;
    // using stm32::usb_class_poll;
    // using stm32::usb_class_sof;
    // using stm32::usb_class_ep0_in;
    // using stm32::usb_class_ep0_out;

    if (USB->ISTR & USB_ISTR_CTR) {
        do {
            uint8_t epnum = USB->ISTR & USB_ISTR_EP_ID;
            if (USB_EPx(epnum) & USB_EP_CTR_RX) { //OUT
                epfunc_out[epnum](epnum);
                ENDP_CTR_RX_CLR(epnum);
            }
            if(USB_EPx(epnum) & USB_EP_CTR_TX){//IN
                epfunc_in[epnum](epnum | 0x80);
                ENDP_CTR_TX_CLR(epnum);
            }
        } while(USB->ISTR & USB_ISTR_CTR);
        return;
    }
    
    if (USB->ISTR & USB_ISTR_SOF) {
        usb_class_sof();
        USB->ISTR = (uint16_t)~USB_ISTR_SOF;
        return;
    }
    
    if (USB->ISTR & USB_ISTR_RESET) {
        usb_class_disconnect();

        #ifdef USBLIB_SOF_ENABLE
        USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SOFM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
        #else
        USB->CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
        #endif
        
        lastaddr = LASTADDR_DEFAULT;
        USB->DADDR = USB_DADDR_EF;
        for (uint16_t i = 0; i < STM32ENDPOINTS; i++) {
            epfunc_in[i]  = endp_callback_default;
            epfunc_out[i] = endp_callback_default;
        }
        
        // state is default - wait for enumeration
        USB->ISTR = (uint16_t)~USB_ISTR_RESET;
        usb_ep_init(0x00, USB_ENDP_CTRL, USB_EP0_BUFSZ, ep0_out);
        usb_ep_init(0x80, USB_ENDP_CTRL, USB_EP0_BUFSZ, ep0_in);
        ep0_buf = nullptr;
        usb_class_init();
    }
    
    if (USB->ISTR & USB_ISTR_SUSP) { // suspend -> still no connection, may sleep
        usb_class_disconnect();
        USB->CNTR |= USB_CNTR_FSUSP | USB_CNTR_LPMODE;
        USB->ISTR = (uint16_t)~USB_ISTR_SUSP;
    }
    
    if (USB->ISTR & USB_ISTR_WKUP) { // wakeup
        USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LPMODE); // clear suspend flags
        USB->ISTR = (uint16_t)~USB_ISTR_WKUP;
    }
}
*/

extern "C" void USB_LP_IRQHandler () {
    GPIOB->ODR |= 1 << 6; // не заходит в прерывание
}
