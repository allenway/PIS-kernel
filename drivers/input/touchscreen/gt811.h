/* drivers/input/touchscreen/gt811.h
 *
 * Copyright (C) 2010 - 2011 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 *Any problem,please contact andrew@goodix.com,+86 755-33338828
 *
 */

#ifndef 	_LINUX_GT811_H
#define		_LINUX_GT811_H

#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#define E_ANDROID	0
#define E_LINUX	1
#define SYSTYPE E_LINUX	
//*************************TouchScreen Work Part*****************************
#define GOODIX_I2C_NAME "cap_ts"
#define GT801_PLUS
#define GT801_NUVOTON
#define GUITAR_UPDATE_STATE 0x02

//define resolution of the touchscreen
#define TOUCH_MAX_HEIGHT 	800
#define TOUCH_MAX_WIDTH		480
//#define STOP_IRQ_TYPE                     // if define then   no stop irq in irq_handle   kuuga add 1202S
#define REFRESH 0     //0~0x64   Scan rate = 10000/(100+REFRESH)//define resolution of the LCD

//#define SHUTDOWN_PORT		IMX_GPIO_NR(1, 16)//S5PV210_GPD0(3)
#define INT_PORT		//	IMX_GPIO_NR(7, 13)//S5PV210_GPH1(6)
#ifdef INT_PORT
//	#define TS_INT 		        gpio_to_irq(INT_PORT)			//Interrupt Number,EINT18(119)
//	#define INT_CFG    	      S3C_GPIO_SFN(3) 					//IO configer as EINT
//#define TS_INT 		gpio_to_irq(INT_PORT)		//Interrupt Number,EINT18 as 119
//#define  INT_CFG    S3C_GPIO_SFN(0xf)			//IO configer,EINT type
#else
	#define TS_INT	0
#endif

/////////////////////////////// UPDATE STEP 5 START /////////////////////////////////////////////////////////////////
#define TPD_CHIP_VERSION_C_FIRMWARE_BASE 0x5A
#define TPD_CHIP_VERSION_D1_FIRMWARE_BASE 0x7A
#define TPD_CHIP_VERSION_E_FIRMWARE_BASE 0x9A
#define TPD_CHIP_VERSION_D2_FIRMWARE_BASE 0xBA


/////////////////////////////// UPDATE STEP 5 END /////////////////////////////////////////////////////////////////

#define FLAG_UP		0
#define FLAG_DOWN		1
//set GT801 PLUS trigger mode,只能设置0或1
//#define INT_TRIGGER		1	   // 1=rising 0=falling
#define POLL_TIME		10	//actual query spacing interval:POLL_TIME+6
//#if(SYSTYPE == E_ANDROID)
//#define GOODIX_MULTI_TOUCH
//#endif
//#ifdef GOODIX_MULTI_TOUCH
//	#define MAX_FINGER_NUM	5//5
//#else
	#define MAX_FINGER_NUM	1
//#endif

#if defined(INT_PORT)
	#if MAX_FINGER_NUM <= 3
	#define READ_BYTES_NUM 2+2+MAX_FINGER_NUM*5
	#elif MAX_FINGER_NUM == 4
	#define READ_BYTES_NUM 2+28
	#elif MAX_FINGER_NUM == 5
	#define READ_BYTES_NUM 2+34
	#endif
#else
	#define READ_BYTES_NUM 2+34
#endif

//#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

#define READ_TOUCH_ADDR_H 0x07
#define READ_TOUCH_ADDR_L 0x21				//GT811 0x721
#define READ_KEY_ADDR_H 0x07
#define READ_KEY_ADDR_L 0x21
#define READ_COOR_ADDR_H 0x07
#define READ_COOR_ADDR_L 0x22
#define READ_ID_ADDR_H 0x00
#define READ_ID_ADDR_L 0xff
//****************************升级模块参数******************************************

//******************************************************************************
struct goodix_ts_data {
	uint16_t addr;
	uint8_t bad_data;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_reset;		//use RESET flag
	int use_irq;		//use EINT flag
	int read_mode;		//read moudle mode,20110221 by andrew
	struct hrtimer timer;
	struct work_struct  work;
	char phys[32];
	int retry;
	int irq;
	spinlock_t				irq_lock;      //add by kuuga
	int 				 irq_is_disable; /* 0: irq enable */ //add by kuuga
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t int_trigger_type;
	uint8_t btn_state;                    // key states
/////////////////////////////// UPDATE STEP 6 START /////////////////////////////////////////////////////////////////
       unsigned int version;
/////////////////////////////// UPDATE STEP 6 END /////////////////////////////////////////////////////////////////

	struct early_suspend early_suspend;
	int (*power)(struct goodix_ts_data * ts, int on);
};

//*****************************End of Part I *********************************

//*************************Touchkey Surpport Part*****************************
#if(SYSTYPE == E_ANDROID)
#define HAVE_TOUCH_KEY//lhh del
#define READ_KEY_VALUE
#define READ_KEY_COOR
#endif
#ifdef HAVE_TOUCH_KEY
	const uint16_t touch_key_array[]={
									  KEY_MENU,				//MENU
									  KEY_HOME,
									  KEY_BACK,
									  KEY_SEARCH
									 };
	#define MAX_KEY_NUM	 (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

//#define COOR_TO_KEY
    #ifdef COOR_TO_KEY

    #define KEY_X       40
    #define KEY_Y       20
    #if 0
    #define AREA_X      0
    #else
    #define AREA_Y      800
    #endif

    enum {x, y};
    s32 key_center[MAX_KEY_NUM][2] = {

	{48,840},{124,840},{208,840},{282,840}

                           };

    #endif

//*****************************End of Part II*********************************

//*****************************End of Part III********************************
/////////////////////////////// UPDATE STEP 7 END /////////////////////////////////////////////////////////////////

struct goodix_i2c_rmi_platform_data {
	uint32_t version;	/* Use this entry for panels with */
	//reservation
};

#define RAW_DATA_READY          1
#define RAW_DATA_NON_ACTIVE     0xffffffff
#define RAW_DATA_ACTIVE         0


enum CHIP_TYPE
{
    GT800 = 1,
    GT800PLUS,
    GT800PLUS3,
    GT816,
    GT811,
    GT8105,
    GT8110,
    GT818PLUS
};


#endif /* _LINUX_GOODIX_TOUCH_H */
