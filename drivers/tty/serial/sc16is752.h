#ifndef __SC16IS752_H__
#define __SC16IS752_H__


#define THR		 	(0x00<<3)		// transmit FIFO, write only
#define RHR			(0x00<<3)		// receive FIFO, read only
#define IER			(0x01<<3)		// interrup enable reg., R/W
#define FCR			(0x02<<3)		// FIFO control, write only
#define IIR			(0x02<<3)		// interrupt status, read only
#define LCR			(0x03<<3)		// line control, R/W
#define MCR			(0x04<<3)		// modem control, R/W
#define LSR			(0x05<<3)		// Line status, R/W
#define MSR			(0x06<<3)		// Modem status, R/W
#define SPR			(0x07<<3)		// scratch pad, R/W
#define TCR			(0x06<<3)		// Transmission Control Register, R/W
#define TLR			(0x07<<3)		// Trigger Level Register, R/W
#define TXFIFO		(0x08<<3)		// TX FIFO, R
#define RXFIFO		(0x09<<3)		// RX FIFO, R
#define DLAB		(0x80<<3)		// Way to swap mem banks

#define IODIR		(0x0A<<3)		// IO Direction Control R/W
#define IOSTATE		(0x0B<<3)		// IO State R/W
#define IOINTMSK	(0x0C<<3)		// IO Interrupt Mask R/W
//#define Reserved		0x0D	// Reserved
#define IOCTRL		(0x0E<<3)		// IO Control R/W
#define EFCR		(0x0F<<3)		//  Enhancede Function Reg R/W

/* special registers, LCR.7 must be set to 1 to R/W to these registers */
#define DLL			(0x00<<3)		// baud rate divisor, LSB, R/W
#define DLH			(0x01<<3)		// baud rate divisor, MSR, R/W
#define EFR			(0x02<<3)		// enhanced register, R/W
#define XON1		(0x04<<3)		// flow control
#define XON2		(0x05<<3)		// flow control
#define XOFF1		(0x06<<3)		// flow control
#define XOFF2		(0x07<<3)		// flow control

#define CHAN_B (1<<1)

#define SC16IS752_LCON_CS5 	0x00
#define SC16IS752_LCON_CS6 	0x01
#define SC16IS752_LCON_CS7 	0x02
#define SC16IS752_LCON_CS8 	0x03

#define SC16IS752_LCON_STOPB	0x04
#define SC16IS752_LCON_PODD		(0x1<<3)
#define SC16IS752_LCON_PEVEN	(0x3<<3)
#define SC16IS752_LCON_PNONE	(0x0<<3)
#define SC16IS752_UFCON_FIFOMODE	(0x7)
#define SC16IS752_UFCON_RXTRIG8 	(0x0<<4)
#define SC16IS752_UFCON_RXTRIG16 	(0x5<<4)
#define SC16IS752_UFCON_RXTRIG60 	(0xc<<4)
#define SC16IS752_TLR_RXTRIG16 	(0x44)
#define SC16IS752_TLR_RXTRIG60 	(0xff)
#define SC16IS752_UFSTAT_TXMASK	(0x7f)
#define SC16IS752_UFSTAT_RXFULL	(0x10)

#define SC16IS752_ENABLE_FIFO		0xc0
#define SC16IS752_TxFIFO_EMPTY		64
#define SC16IS752_TxFIFO_FULL		0
#define SC16IS752_RxFIFO_EMPTY		0
#define SC16IS752_RxFIFO_FULL		64

#define SC16IS752_RX_INT				0x4
#define SC16IS752_TX_INT				0x2
#define SC16IS752_RXTIMEOUT_INT		0xc

#define NO_INT_PENDING					0x1

#define BAUD_RATE_CLOCK					14745600  /*3.6864MHz*/
#define BAUD_RATE						115200

#define ENABLE_RX_INT				0x01
#define ENABLE_TX_INT				0x02
#define ENABLE_RLS_INT				0x04
#define ENABLE_MS_INT				0x08

#define DISABLE_RX_TX				0x06
#define DISABLE_RX					0x02
#define DISABLE_TX					0x04
#define DISABLE_FIFO					0x0

#define ENABLE_TCR_TLR				(1<<2)
#define ENABLE_IRDA_MODE			(1<<6)
#define ENABLE_LOOPBACK			(1<<4)

#define RESET_FIFO					(0x06)

#define SC16IS752_UERSTAT_BREAK		0x10
#define SC16IS752_UERSTAT_PARITY		0x04
#define SC16IS752_UERSTAT_FRAME		0x08
#define SC16IS752_UERSTAT_OVERRUN		0x02
#define SC16IS752_LSR_DR		0x01
#define SC16IS752_UERSTAT_ANY	(SC16IS752_UERSTAT_BREAK|SC16IS752_UERSTAT_FRAME|SC16IS752_UERSTAT_OVERRUN)

#endif /*__SC16IS752_H__*/