# stm32f103_xvcusb
Xilinx Virtual Cable implemented as a USB device on an STM32F103

This code provides a minimal usb device with 1 Bulk in and 1 Bulk out endpoint over
which [XVC](https://github.com/Xilinx/XilinxVirtualCable) protocol 1.0 is implemented.

The device can be made to show up as /dev/ttyUXBx under linux with the following commands
  sudo modprobe usbserial vendor=0x0483 product=0x5740 
  
After which you can run an XVC server daemon with the command
  netcat -l 2542 < /dev/ttyUSBx > /dev/ttyUSBx  
 
## Pinout

|Pin| Name | Function   | DIR |  Electrical  |  Connected to       |
|--:|------|------------|-----|--------------|---------------------|
| 14| PA0  | TIM2 CH1   | out | AF_PP 10MHz  | Xilinx JTAG TCK     |
| 15| PA1  | GPIO A1    | out | OUT_PP 10MHz | Xilinx JTAG TDI     |
| 16| PA2  | GPIO A2    | in  | PullUp       | Xilinx JTAG TDO     |
| 17| PA3  | GPIO A3    | out | OUT_PP 10MHz | Xilinx JTAG TMS     |
|   | PA9  | USART1 TX  | out | AF_PP 10MHz  | console debug out   |
|   | PC13 | GPIO C13   | out | OUT_OD 2MHz  | LED (active low)    |


PA0-3 form the Xilinx JTAG connection.
PA9 is a 921600 8N1 serial debug output
PC13 toggles on each XVC connection.

