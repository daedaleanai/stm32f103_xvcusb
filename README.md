# stm32f103_xvcusb
Xilinx Virtual Cable implemented as a USB device on an STM32F103

This code provides a minimal usb device with 1 Bulk in and 1 Bulk out endpoint over
which [XVC](https://github.com/Xilinx/XilinxVirtualCable) protocol 1.0 is implemented.

The device can be made to show up as /dev/ttyUXBx under linux with the following commands
```
  sudo modprobe usbserial vendor=0x0483 product=0x5740 
```

After which you can run an XVC server daemon with the command
```
  netcat -l 2542 < /dev/ttyUSBx > /dev/ttyUSBx  
```

## Pinout STM32F103CB (LQFP48/LQFP48)

| Pin | Name | Function  | DIR    | Electrical   | Connected to       |
|----:|:-----|:----------|:-------|:-------------|:-------------------|
| 10  | PA0  | TIM2 CH1  | out    | AF_PP 50MHz  | Xilinx JTAG TCK    |
| 11  | PA1  | GPIO A1   | out    | OUT_PP 50MHz | Xilinx JTAG TDI    |
| 12  | PA2  | GPIO A2   | in     | PullUp       | Xilinx JTAG TDO    |
| 13  | PA3  | GPIO A3   | out    | OUT_PP 50MHz | Xilinx JTAG TMS    |
| 30  | PA9  | USART1 TX | out    | AF_PP 50MHz  | console debug out  |
| 32  | PA11 | USB D-    |        |              | automatic          |
| 33  | PA12 | USB D+    |        |              | automatic          |
| 34  | PA13 | SWDIO     | in/out |              | ST-Link programmer |
| 37  | PA14 | SWCLK     | in/out |              | ST-Link programmer |
|  2  | PC13 | GPIO C13  | out    | OUT_OD 2MHz  | LED (active low)   |


PA0-3 form the Xilinx JTAG connection.

PA9 is a 921600 8N1 serial debug output.

PC13 toggles on each XVC command.

