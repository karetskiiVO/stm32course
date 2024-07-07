#ifndef USB_h
#define USB_h 

#include <stm32l1xx.h>
#include <stm32l152xc.h>
#include <system_stm32l1xx.h>

namespace stm32 {

void USBInit ();
extern "C" void USB_LP_IRQHandler (void);

}
#endif // usb.h