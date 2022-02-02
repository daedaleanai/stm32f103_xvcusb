#pragma once
/*
   Low level driver for STM32F103 USB.

   See RM0008 STM32F10x Reference manual DocID13902 Rev 16 November 2015   Section 23 p.625
   https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

   This driver requires the following entries in the linker script:

       USB     = 0x40005C00;
       USB_PMA = 0x40006000;

   Relevant interrupts:

       USB_HP_CAN_TX_IRQn  = 19, // USB High Priority (or CAN TX) interrupts -> double-buffered BULK and ISO CTR
       USB_LP_CAN_RX0_IRQn = 20, // USB Low Priority (or CAN RX0) interrupts -> all ISTR interupts
       USB_FS_WKUP_IRQn    = 42, // USB Device FS Wakeup through EXTI line 18 interrupt
*/

#include <stdint.h>

// Universal serial bus full-speed device interface
// Sec 23.5: The peripheral registers can be accessed by half-words (16-bit) or words (32-bit).
extern struct USB_Type {
		volatile uint32_t EPR[8];
		const uint8_t RESERVED[32];
        volatile uint32_t CNTR;          // @40 Control Register
        volatile uint32_t ISTR;          // @44 Interrupt Status Register
        volatile uint32_t FNR;           // @48 Frame Number Register
        volatile uint32_t DADDR;         // @4C Device Address
        volatile uint32_t BTABLE;        // @50 Buffer Table. this driver assumes this is set to zero
} USB;		// @0x40005C00 


// USB->EPRx endpoint registers
enum {
	USB_EPRx_CTR_RX 	= 1UL<<15, 				// Correct transfer for reception
	USB_EPRx_DTOG_RX 	= 1UL<<14, 				// Data Toggle, for reception transfers
	USB_EPRx_STAT_RX 	= ((1UL<<2)-1) << 12, 	// Status bits, for reception transfers, write 1 to toggle
	USB_EPRx_SETUP 		= 1UL<<11, 				// Setup transaction completed (r/o)
	USB_EPRx_EP_TYPE 	= ((1UL<<2)-1) << 9, 	// Endpoint type
	USB_EPRx_EP_KIND 	= 1UL<<8, 				// Endpoint kind: for TYPE==BULK: DBL_BUF,  for TYPE==CONTROL: STATUS_OUT
	USB_EPRx_EP_TYPEKIND = ((1UL<<3)-1) << 8,
	USB_EPRx_CTR_TX 	= 1UL<<7, 				// Correct Transfer for transmission
	USB_EPRx_DTOG_TX 	= 1UL<<6, 				// Data Toggle, for transmission transfers
	USB_EPRx_STAT_TX 	= ((1UL<<2)-1) << 4, 	// Status bits, for transmission transfers, write 1 to toggle 
	USB_EPRx_EA 		= ((1UL<<4)-1),    	    // Endpoint address		

	// Values for USB_EPRx_STAT_xx
	USB_EP_STAT_DISABLED = 0,
	USB_EP_STAT_STALL    = 1,
	USB_EP_STAT_NAK      = 2,
	USB_EP_STAT_VALID    = 3,

	// Values for USB_EPRx_TYPE|KIND
	USB_EP_TYPE_BULK           = 0,
	USB_EP_TYPE_BULK_DBLBUF    = 1,
	USB_EP_TYPE_CONTROL        = 2,
	USB_EP_TYPE_CONTROL_STSOUT = 3,
	USB_EP_TYPE_ISO            = 4,
	USB_EP_TYPE_INTERRUPT      = 6,

	USB_EP_CFG = USB_EPRx_EP_TYPEKIND|USB_EPRx_EA, // bits to preserve when toggling or clearing
	USB_EP_CTR = USB_EPRx_CTR_RX|USB_EPRx_CTR_TX,  // bits to write as 1 to preserve
};

// because of the toggles and w0c bits, none of these are straightforward

// Clear CTR_xX, set type, kind and endpoint address.  Does not touch the toggle/stat bits. 
inline void usb_ep_config(uint8_t ep, uint16_t typekind, uint8_t ea) { 
	USB.EPR[ep & 0x7f] = ((typekind<<8) & USB_EPRx_EP_TYPEKIND) | (ea & USB_EPRx_EA); 
}

inline void usb_ep_setkind(uint8_t ep) { 
	USB.EPR[ep & 0x7f] = (USB.EPR[ep & 0x7f] & (USB_EPRx_EA|USB_EPRx_EP_TYPE)) | USB_EP_CTR | USB_EPRx_EP_KIND; 
}

// Clear endpoint address and resets typekind to zero (BULK), clears DTOG_xX and STAT_xX to DISABLED
inline void usb_ep_reset(uint8_t ep) { 
	USB.EPR[ep & 0x7f] = USB.EPR[ep & 0x7f] & (USB_EPRx_DTOG_RX|USB_EPRx_STAT_RX|USB_EPRx_DTOG_TX|USB_EPRx_STAT_TX); 
}


inline void 	usb_ep_set_stat_rx(uint8_t ep, uint16_t val) { USB.EPR[ep & 0x7f] = USB_EP_CTR | ((USB.EPR[ep & 0x7f] & (USB_EP_CFG|USB_EPRx_STAT_RX)) ^ ((val << 12)&USB_EPRx_STAT_RX)); }
inline void 	usb_ep_set_stat_tx(uint8_t ep, uint16_t val) { USB.EPR[ep & 0x7f] = USB_EP_CTR | ((USB.EPR[ep & 0x7f] & (USB_EP_CFG|USB_EPRx_STAT_TX)) ^ ((val << 4)&USB_EPRx_STAT_TX)); }
inline void     usb_ep_clr_ctr_rx( uint8_t ep) { USB.EPR[ep & 0x7f] = USB_EPRx_CTR_TX | (USB.EPR[ep & 0x7f] & USB_EP_CFG); }
inline void     usb_ep_clr_ctr_tx( uint8_t ep) { USB.EPR[ep & 0x7f] = USB_EPRx_CTR_RX | (USB.EPR[ep & 0x7f] & USB_EP_CFG); }
inline void     usb_ep_clr_dtog_rx(uint8_t ep) { USB.EPR[ep & 0x7f] = USB_EP_CTR | (USB.EPR[ep & 0x7f] & (USB_EP_CFG|USB_EPRx_DTOG_RX)); }
inline void     usb_ep_set_dtog_rx(uint8_t ep) { USB.EPR[ep & 0x7f] = USB_EP_CTR | ((USB.EPR[ep & 0x7f] & (USB_EP_CFG|USB_EPRx_DTOG_RX)) ^ USB_EPRx_DTOG_RX); }
inline void     usb_ep_clr_dtog_tx(uint8_t ep) { USB.EPR[ep & 0x7f] = USB_EP_CTR | (USB.EPR[ep & 0x7f] & (USB_EP_CFG|USB_EPRx_DTOG_TX)); }
inline uint16_t usb_ep_get_stat_rx(uint8_t ep) { return (USB.EPR[ep & 0x7f] & USB_EPRx_STAT_RX) >> 12 ; }
inline uint16_t usb_ep_get_stat_tx(uint8_t ep) { return (USB.EPR[ep & 0x7f] & USB_EPRx_STAT_TX) >> 4 ; }
inline uint16_t usb_ep_get_ep_typekind(uint8_t ep) { return (USB.EPR[ep & 0x7f] & USB_EPRx_EP_TYPEKIND) >> 8 ; }
inline uint16_t usb_ep_get_ea(     uint8_t ep) { return (USB.EPR[ep & 0x7f] & USB_EPRx_EA) >> 0 ; }

inline int usb_ep_is_tx(uint8_t ep) { return (ep & 0x80) != 0; } // true: IN/TX,  false: OUT/RX

// USB->CNTR control register
enum {
	USB_CNTR_CTRM 		= 1UL<<15, // Correct transfer interrupt mask
	USB_CNTR_PMAOVRM 	= 1UL<<14, // Packet memory area over / underrun interrupt mask
	USB_CNTR_ERRM 		= 1UL<<13, // Error interrupt mask
	USB_CNTR_WKUPM 		= 1UL<<12, // Wakeup interrupt mask
	USB_CNTR_SUSPM 		= 1UL<<11, // Suspend mode interrupt mask
	USB_CNTR_RESETM 	= 1UL<<10, // USB reset interrupt mask
	USB_CNTR_SOFM 		= 1UL<<9, // Start of frame interrupt mask
	USB_CNTR_ESOFM 		= 1UL<<8, // Expected start of frame interrupt mask
	USB_CNTR_RESUME 	= 1UL<<4, // Resume request
	USB_CNTR_FSUSP 		= 1UL<<3, // Force suspend
	USB_CNTR_LPMODE 	= 1UL<<2, // Low-power mode
	USB_CNTR_PDWN 		= 1UL<<1, // Power down
	USB_CNTR_FRES 		= 1UL<<0, // Force USB Reset		
};

// USB->ISTR interrupt status register
enum {
	USB_ISTR_CTR 	= 1UL<<15, // Correct transfer
	USB_ISTR_PMAOVR = 1UL<<14, // Packet memory area over / underrun
	USB_ISTR_ERR 	= 1UL<<13, // Error
	USB_ISTR_WKUP 	= 1UL<<12, // Wakeup
	USB_ISTR_SUSP 	= 1UL<<11, // Suspend mode request
	USB_ISTR_RESET 	= 1UL<<10, // reset request
	USB_ISTR_SOF 	= 1UL<<9,  // start of frame
	USB_ISTR_ESOF 	= 1UL<<8,  // Expected start frame
	USB_ISTR_DIR 	= 1UL<<4,  // Direction of transaction
	USB_ISTR_EP_ID 	= ((1UL<<4)-1) << 0, // Endpoint Identifier		
};

inline uint16_t usb_istr_get_ep_id() { return (USB.ISTR & USB_ISTR_EP_ID) >> 0 ; }

// USB->FNR frame number register
enum {
	USB_FNR_RXDP = 1UL<<15, // Receive data + line status
	USB_FNR_RXDM = 1UL<<14, // Receive data - line status
	USB_FNR_LCK  = 1UL<<13, // Locked
	USB_FNR_LSOF = ((1UL<<2)-1) << 11, // Lost SOF
	USB_FNR_FN   = ((1UL<<11)-1) << 0, // Frame number		
};
inline uint16_t usb_fnr_get_lsof() { return (USB.FNR & USB_FNR_LSOF) >> 11 ; }
inline uint16_t usb_fnr_get_fn()   { return (USB.FNR & USB_FNR_FN) >> 0 ; }

// USB->DADDR device address
enum {
	USB_DADDR_EF 	= 1UL<<7, 				// Enable function
	USB_DADDR_ADD 	= ((1UL<<7)-1) << 0, 	// Device address		
};
inline void usb_daddr_set_add(uint16_t val) { USB.DADDR = (USB.DADDR & ~USB_DADDR_ADD) | ((val<<0) & USB_DADDR_ADD); }
inline uint16_t usb_daddr_get_add() { return (USB.DADDR & USB_DADDR_ADD) >> 0 ; }

extern union {
    struct {
        volatile uint32_t ADDR_TX;  // in units of uint16, always even
        volatile uint32_t COUNT_TX;
        volatile uint32_t ADDR_RX;
        volatile uint32_t COUNT_RX;
    } btable[8];  // located here by virtue of USB.BTABLE being zero
    uint16_t buf[512]; // interspersed, 2 bytes data, 2 bytes reserved, only accessible as uint16 or uint32
} USB_PMA; // @ 0x50006000

enum {
	USB_PMA_COUNT_BLSIZE 	= 1UL<<15,
	USB_PMA_COUNT_NUMBLOCKS = ((1UL<<5)-1) << 10,
	USB_PMA_COUNT_COUNT 	= ((1UL<<10)-1) << 0,
};

// tx/rx buffers are only accessible as uint16_t, not as bytes!
inline uint16_t* usb_ep_tx_buf(int ep) { return USB_PMA.buf + USB_PMA.btable[ep].ADDR_TX; }
inline uint16_t* usb_ep_rx_buf(int ep) { return USB_PMA.buf + USB_PMA.btable[ep].ADDR_RX; }

// these are in units of bytes
inline void     usb_ep_set_tx_count(int ep, uint16_t len) { USB_PMA.btable[ep].COUNT_TX = len & USB_PMA_COUNT_COUNT; }
inline uint16_t usb_ep_get_tx_count(int ep) { return USB_PMA.btable[ep].COUNT_TX & USB_PMA_COUNT_COUNT; }
inline uint16_t usb_ep_get_rx_count(int ep) { return USB_PMA.btable[ep].COUNT_RX & USB_PMA_COUNT_COUNT; }

inline uint16_t usb_ep_get_rx_size(int ep) { 
	uint16_t crx = USB_PMA.btable[ep].COUNT_RX;
	uint16_t v = (crx & USB_PMA_COUNT_NUMBLOCKS)>>10;
	if (crx & USB_PMA_COUNT_BLSIZE)
			return (v+1) * 32;
	return v * 2; 
}

inline void usb_ep_set_rx_size(int ep, uint16_t size) { 
	if (size < 62) {
		USB_PMA.btable[ep].COUNT_RX = (size/2) << 10;
	} else {
		USB_PMA.btable[ep].COUNT_RX = (((size/32)-1) << 10) | USB_PMA_COUNT_BLSIZE;
	}
}
