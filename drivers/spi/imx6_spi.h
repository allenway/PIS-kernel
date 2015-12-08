#ifndef _TQ6410_SPI_H_
#define _TQ6410_SPI_H_

#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>

//#include <mach/map.h>
#include <mach/gpio.h>
//#include <plat/gpio-cfg.h>
//#include <mach/regs-gpio.h>

#include <linux/ioport.h>
#include <linux/io.h>
#include <asm/io.h>

//#include <mach/regs-clock.h>
//#include <plat/clock.h>

/* Registers and bit-fields */

#define IMX6_CH_CFG_CH0			0x200c000
#define IMX6_CLK_CFG_CH0		(IMX6_CH_CFG_CH0+0x4)
#define IMX6_MODE_CFG_CH0		(IMX6_CH_CFG_CH0+0x8)
#define IMX6_CS_REG_CH0			(IMX6_CH_CFG_CH0+0xc)
#define IMX6_SPI_INT_EN_CH0		(IMX6_CH_CFG_CH0+0x10)
#define IMX6_CS_SPI_STATUS_CH0		(IMX6_CH_CFG_CH0+0x14)
#define IMX6_SPI_TX_DATA_CH0		(IMX6_CH_CFG_CH0+0x18)
#define IMX6_SPI_RX_DATA_CH0		(IMX6_CH_CFG_CH0+0x1c)
#define IMX6_PACKET_CNT_REG_CH0		(IMX6_CH_CFG_CH0+0x20)
#define IMX6_PENDING_CLR_REG_CH0	(IMX6_CH_CFG_CH0+0x24)
#define IMX6_SWAP_CFG_CH0		(IMX6_CH_CFG_CH0+0x28)
#define IMX6_FB_CLK_SEL_CH0		(IMX6_CH_CFG_CH0+0x2c)

#define IMX6_CH_CFG_CH1		0xE1400000
#define IMX6_CLK_CFG_CH1		(IMX6_CH_CFG_CH1+0x4)
#define IMX6_MODE_CFG_CH1		(IMX6_CH_CFG_CH1+0x8)
#define IMX6_CS_REG_CH1		(IMX6_CH_CFG_CH1+0xc)
#define IMX6_SPI_INT_EN_CH1		(IMX6_CH_CFG_CH1+0x10)
#define IMX6_CS_SPI_STATUS_CH1	(IMX6_CH_CFG_CH1+0x14)
#define IMX6_SPI_TX_DATA_CH1		(IMX6_CH_CFG_CH1+0x18)
#define IMX6_SPI_RX_DATA_CH1		(IMX6_CH_CFG_CH1+0x1c)
#define IMX6_PACKET_CNT_REG_CH1	(IMX6_CH_CFG_CH1+0x20)
#define IMX6_PENDING_CLR_REG_CH1	(IMX6_CH_CFG_CH1+0x24)
#define IMX6_SWAP_CFG_CH1		(IMX6_CH_CFG_CH1+0x28)
#define IMX6_FB_CLK_SEL_CH1		(IMX6_CH_CFG_CH1+0x2c)



#define IMX6_SPI_CH_HS_EN		(1<<6)	/* High Speed Enable */
#define IMX6_SPI_CH_HS_DISEN		(0<<6)
#define IMX6_SPI_CH_SW_RST		(1<<5)
#define IMX6_SPI_CH_SLAVE		(1<<4)
#define IMX6_SPI_CH_MASTER		(0<<4)
#define IMX6_SPI_CPOL_L		(1<<3)
#define IMX6_SPI_CPHA_B		(1<<2)
#define IMX6_SPI_CH_RXCH_OFF		(0<<1)
#define IMX6_SPI_CH_TXCH_OFF		(0<<0)
#define IMX6_SPI_CH_RXCH_ON		(1<<1)
#define IMX6_SPI_CH_TXCH_ON		(1<<0)

#define IMX6_SPI_CLKSEL_SRCMSK	(3<<9)
#define IMX6_SPI_CLKSEL_SRCSHFT	9
#define IMX6_SPI_ENCLK_ENABLE	(1<<8)
#define IMX6_SPI_PSR_MASK 		0xff

#define IMX6_SPI_MODE_CH_TSZ_BYTE		(0<<29)
#define IMX6_SPI_MODE_CH_TSZ_HALFWORD	(1<<29)
#define IMX6_SPI_MODE_CH_TSZ_WORD		(2<<29)
#define IMX6_SPI_MODE_CH_TSZ_MASK		(3<<29)
#define IMX6_SPI_MODE_BUS_TSZ_BYTE		(0<<17)
#define IMX6_SPI_MODE_BUS_TSZ_HALFWORD	(1<<17)
#define IMX6_SPI_MODE_BUS_TSZ_WORD		(2<<17)
#define IMX6_SPI_MODE_BUS_TSZ_MASK		(3<<17)
#define IMX6_SPI_MODE_RXDMA_ON		(1<<2)
#define IMX6_SPI_MODE_TXDMA_ON		(1<<1)
#define IMX6_SPI_MODE_4BURST			(1<<0)

#define IMX6_SPI_SLAVE_AUTO			(1<<1)
#define IMX6_SPI_SLAVE_SIG_INACT		(1<<0)


#define IMX6_SPI_INT_TRAILING_EN		(1<<6)
#define IMX6_SPI_INT_RX_OVERRUN_EN		(1<<5)
#define IMX6_SPI_INT_RX_UNDERRUN_EN		(1<<4)
#define IMX6_SPI_INT_TX_OVERRUN_EN		(1<<3)
#define IMX6_SPI_INT_TX_UNDERRUN_EN		(1<<2)
#define IMX6_SPI_INT_RX_FIFORDY_EN		(1<<1)
#define IMX6_SPI_INT_TX_FIFORDY_EN		(1<<0)

#define IMX6_SPI_ST_RX_OVERRUN_ERR		(1<<5)
#define IMX6_SPI_ST_RX_UNDERRUN_ERR	(1<<4)
#define IMX6_SPI_ST_TX_OVERRUN_ERR		(1<<3)
#define IMX6_SPI_ST_TX_UNDERRUN_ERR	(1<<2)
#define IMX6_SPI_ST_RX_FIFORDY		(1<<1)
#define IMX6_SPI_ST_TX_FIFORDY		(1<<0)

#define IMX6_SPI_PACKET_CNT_EN		(1<<16)

#define IMX6_SPI_PND_TX_UNDERRUN_CLR		(1<<4)
#define IMX6_SPI_PND_TX_OVERRUN_CLR		(1<<3)
#define IMX6_SPI_PND_RX_UNDERRUN_CLR		(1<<2)
#define IMX6_SPI_PND_RX_OVERRUN_CLR		(1<<1)
#define IMX6_SPI_PND_TRAILING_CLR		(1<<0)

#define IMX6_SPI_SWAP_RX_HALF_WORD		(1<<7)
#define IMX6_SPI_SWAP_RX_BYTE		(1<<6)
#define IMX6_SPI_SWAP_RX_BIT			(1<<5)
#define IMX6_SPI_SWAP_RX_EN			(1<<4)
#define IMX6_SPI_SWAP_TX_HALF_WORD		(1<<3)
#define IMX6_SPI_SWAP_TX_BYTE		(1<<2)
#define IMX6_SPI_SWAP_TX_BIT			(1<<1)
#define IMX6_SPI_SWAP_TX_EN			(1<<0)

#define IMX6_SPI_FBCLK_MSK		(3<<0)

#define SUSPND    (1<<0)
#define SPIBUSY   (1<<1)
#define RXBUSY    (1<<2)
#define TXBUSY    (1<<3)

#define IMX6_SPI_MAX_TRAILCNT	0x3ff
#define IMX6_SPI_TRAILCNT_OFF	19

#define IMX6_SPI_TRAILCNT		IMX6_SPI_MAX_TRAILCNT


#define SMB380_ADDR_WRITE  	0x70 
#define SMB380_ADDR_READ   	0x71 

//Registers  address 
#define  OPER_REG        	0x15//Wake_up(bit 0),Wake_up_pause(bit1,bit2),Shadow_dis(bit3), 
                                //latch_int(bit4),new_data_int(bit5),enable_adv_int(bit6),spi4 mode choose(bit7) 
#define  OPER_SET        	0x14//rang(bit4'3),bandwith(bit2'1'0) 
#define  DUR_HYST        	0x11//any_motion_dur(bit6'7),hg_hyst(bit5'4'3),lg_hyst(bit5'4'3) 
#define  ANY_MOTION_TRES 	0x10 
#define  HG_DUR          	0x0F 
#define  HG_THRES        	0x0E 
#define  LG_DUR          	0x0D 
#define  LG_THRES        	0X0C 
#define  INTERRUPT_SET   	0x0B//enable_lg(bit0),enble_hg(bit1),counter_lg(bit2'3), 
                                //counter_hg(bit4'5),any_motion(bit6),alert(bit7) 
#define  CONTROL_REG     	0x0A//sleep(bit0),soft_reset(bit1),self_test0(bit2),self_test1(bit3), 
                                //ee_w(bit4),updata_image(bit5),reset_int(bit6), 
#define  STATUS_REG      	0x09//status_hg(bit0),status_lg(bit1),hg_latch(bit2),lg_latch(bit3),alert(bit4),st_result(bit7) 
#define  ACC_Z_MSB       	0x07    
#define  ACC_Z_LSB       	0x06//ACC_Z_BIT0'1(bit6'7),new_data_z(bit0) 
#define  ACC_Y_MSB       	0x05 
#define  ACC_Y_LSB       	0x04//ACC_Y_BIT0'1(bit6'7),new_data_y(bit0) 
#define  ACC_X_MSB       	0x03 
#define  ACC_X_LSB       	0x02//ACC_X_BIT0'1(bit6'7),new_data_x(bit0)
#define	 SMB380_VERSION		0x01//版本寄存器 
#define  SMB380_CHIP_ID		0x00//ID寄存器

//0x15
#define  SPI4				(1<<8)
#define  Wake_up			(1<<0)

//0x14
//移3位
#define	 two_G				0
#define  four_G				1
#define  eight_G			2

//Bandwidth
#define  bandWidth_25		0
#define  bandWidth_50		1
#define  bandWidth_100		2
#define  bandWidth_190		3
#define  bandWidth_375		4
#define  bandWidth_750		5
#define  bandWidth_1500		6


//IOCTL操作设置系统参数

#define SPI_CHANNEL_SET			0
#define SPI_BAUDRATE_SET		1
/*#define SPI_CPOL_SET			2
#define SPI_CPHA_SET			3
#define SPI_MASTR_SET			4//主从模式设置
#define SPI_NORMAL_SET			5*/
#define X_AXIS_READ			6
#define Y_AXIS_READ			7
#define Z_AXIS_READ			8
#define SMB380_INTERRUPTMODE		9
#define UPDATE_SMB380_IMG		10
#define IOCTL_S_POLL_INTERVAL	11
#endif
