#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
//#include <mach/regs-adc.h>
//#include <mach/adc.h>
#include <mach/irqs.h>


#include "imx6_spi.h"
#include <linux/input-polldev.h>

#define POLL_INTERVAL  50
#define INPUT_FUZZ 4
#define INPUT_FLAT 4
#define SABRESD_ECSPI2_CS0    IMX_GPIO_NR(5, 29)

#define DEVICE_NAME 	"angle"

#define DEBUG  1
#undef DEBUG
#ifdef DEBUG
#define DPRINTK(format, x... )		printk("e9-SPI: "format, ##x)
#else
#define DPRINTK(format, x... )
#endif
#if 0
#define USER_POLL_READ 1
#else
#undef USER_POLL_READ
#endif
#ifdef USER_POLL_READ
static struct input_polled_dev *g_sensor_input_dev;
#else
static struct input_dev *g_sensor_input_dev;
#endif
static unsigned char channel=0;//选择通道0

static volatile int *spi_ch_cfg = NULL;
static volatile int *spi_clk_cfg = NULL;
static volatile int *spi_mode_cfg = NULL;//
static volatile int *spi_cs_reg = NULL;//
static volatile int *spi_int_en = NULL;
static volatile int *spi_status = NULL;
static volatile int *spi_tx_dat = NULL;//
static volatile int *spi_rx_dat = NULL;
static volatile int *spi_packet_cnt = NULL;
static volatile int *spi_pending_clr = NULL;
static volatile int *usb_pw = NULL;
static volatile int *usb_clk = NULL;
static volatile int *other = NULL;


//枚举类型
typedef enum
{
	Axis_X = 'x',
	Axis_Y = 'y',
	Axis_Z = 'z',
}Axis_Acc;

static struct timer_list timer;
static unsigned int g_poll_interval;

static unsigned int ucSpiBaud=300;//SPI时钟 	KHZ

//#define Enable_nSS(channel)		do{(channel==0)?(imx6_gpio_setpin(IMX6_GPC(3), 0)):(imx6_gpio_setpin(IMX6_GPC(7), 0));}while(0);
//#define Disable_nSS(channel)		do{(channel==0)?(imx6_gpio_setpin(IMX6_GPC(3), 1)):(imx6_gpio_setpin(IMX6_GPC(7), 1));}while(0);

#define Channel_TxOn			do{*spi_ch_cfg |= IMX6_SPI_CH_TXCH_ON;}while(0);
#define Channel_RxOn			do{*spi_ch_cfg |= IMX6_SPI_CH_RXCH_ON;}while(0);
#define Channel_TxOff			do{*spi_ch_cfg &= ~IMX6_SPI_CH_TXCH_ON;}while(0);
#define Channel_RxOff			do{*spi_ch_cfg &= ~IMX6_SPI_CH_RXCH_ON;}while(0);


#ifdef USER_POLL_READ
static void g_sensor_input_dev_poll(struct input_polled_dev *dev);
#else
static void g_sensor_input_dev_poll(int para);
#endif

inline static void gsensor_pin_config(void)//选择0
{
	int i;
	printk("+++gsensor_pin_config+++");
	if(channel == 1)//SPI1
	{
		gpio_direction_output(SABRESD_ECSPI2_CS0, 1);
		gpio_set_value(SABRESD_ECSPI2_CS0, 1);
	}
	else if(channel == 0)//SPI0
	{
		
			gpio_direction_output(SABRESD_ECSPI2_CS0, 1);
			gpio_set_value(SABRESD_ECSPI2_CS0, 1);
		
	}
	
}

//延时
static void delayLoop(unsigned long count)
{
    unsigned int j;
    for(j = 0; j < count; j++);
}


static void gsensor_init(unsigned int baudrate)
{
	unsigned long pclk;//
	struct clk *clk_p;
	clk_p = clk_get(NULL, "pclk_psys");
	pclk  = clk_get_rate(clk_p);//得到时钟频率
	printk("+++pclk=0x%x+++\n", pclk);
	gsensor_pin_config();//选择通道

	if(channel == 0)
	{
		//__raw_writel((__raw_readl(S5P_CLKGATE_IP3) | (1<<12)), S5P_CLKGATE_IP3);//使能SPI时钟
		//__raw_writel((__raw_readl(imx6_SCLK_GATE) & 0x0fffff), imx6_SCLK_GATE);
		//__raw_writel((__raw_readl(imx6_SCLK_GATE) | 0x100000), imx6_SCLK_GATE);
		//__raw_writel((__raw_readl(imx6_CLK_DIV2) | 0x52), imx6_CLK_DIV2);
		//DPRINTK("clkcon=0x%x,  %s\n", __raw_readl(S5P_CLKGATE_IP3),(__raw_readl(S5P_CLKGATE_IP3) & (1<<12))?"SPI0_clock_enable":"SPI0_clock_disable");
		//__raw_writel((__raw_readl(imx6_PCLK_GATE) | imx6_CLKCON_PCLK_SPI0), imx6_PCLK_GATE);
		spi_ch_cfg = (int *)ioremap(IMX6_CH_CFG_CH0, 4);
		spi_clk_cfg = (int *)ioremap(IMX6_CLK_CFG_CH0, 4);
		spi_mode_cfg = (int *)ioremap(IMX6_MODE_CFG_CH0, 4);
		spi_cs_reg = (int *)ioremap(IMX6_CS_REG_CH0, 4);
		spi_int_en = (int *)ioremap(IMX6_SPI_INT_EN_CH0, 4);
		spi_status = (int *)ioremap(IMX6_CS_SPI_STATUS_CH0,4);
		spi_tx_dat = (int *)ioremap(IMX6_SPI_TX_DATA_CH0, 4);
		spi_rx_dat = (int *)ioremap(IMX6_SPI_RX_DATA_CH0, 4);
		spi_packet_cnt = (int *)ioremap(IMX6_PACKET_CNT_REG_CH0, 4);
		spi_pending_clr = (int *)ioremap(IMX6_PENDING_CLR_REG_CH0, 4);
	}
	else if(channel == 1)
	{
		spi_ch_cfg = (int *)ioremap(IMX6_CH_CFG_CH1, 1);
		spi_clk_cfg = (int *)ioremap(IMX6_CLK_CFG_CH1, 2);
		spi_mode_cfg = (int *)ioremap(IMX6_MODE_CFG_CH1, 4);
		spi_cs_reg = (int *)ioremap(IMX6_CS_REG_CH1, 2);
		spi_int_en = (int *)ioremap(IMX6_SPI_INT_EN_CH1, 1);
		spi_status = (int *)ioremap(IMX6_CS_SPI_STATUS_CH1, 3);
		spi_tx_dat = (int *)ioremap(IMX6_SPI_TX_DATA_CH1, 4);
		spi_rx_dat = (int *)ioremap(IMX6_SPI_RX_DATA_CH1, 4);
		spi_packet_cnt = (int *)ioremap(IMX6_PACKET_CNT_REG_CH1, 3);
		spi_pending_clr = (int *)ioremap(IMX6_PENDING_CLR_REG_CH1, 1);
		//__raw_writel((__raw_readl(S5P_CLKGATE_IP3) | (1<<13)), S5P_CLKGATE_IP3);
		//DPRINTK("clkcon=0x%x,  %s\n", __raw_readl(S5P_CLKGATE_IP3),(__raw_readl(S5P_CLKGATE_IP3) & (1<<13))?"SPI0_clock_enable":"SPI0_clock_disable");
	}
	*spi_ch_cfg &= ~(1<<6);
	*spi_ch_cfg |= (IMX6_SPI_CH_MASTER)|(IMX6_SPI_CPOL_L)|(IMX6_SPI_CPHA_B);//enable high speed,master,cpol low,cphb,txch off,rxch off 0xc

	*spi_clk_cfg |= (0<<9)|((pclk/1000/2/baudrate-1)&IMX6_SPI_PSR_MASK)|(1<<8);//pclk is clk source,enable clk;baudrate
	*spi_mode_cfg |= (0<<29)|(0<<17)|(0x0<<5)|(0x0<<11)|(0x3ff<<19);//poll方式 以字节形式发送接受
	*spi_cs_reg |= (0<<1)|(1<<0);//认为片选信号，不输出nss信号
	*spi_int_en = 0;//不选择中断方式
	*spi_packet_cnt = 0;


	DPRINTK("spi_ch_cfg=0x%x, spi_clk_cfg=0x%x, spi_mode_cfg=0x%x, spi_cs_reg=0x%x ,spi_int_en = 0x%x ,spi_status = 0x%x,spi_packet_cnt = 0x%x\n", *spi_ch_cfg, *spi_clk_cfg, *spi_mode_cfg,*spi_cs_reg,*spi_int_en,*spi_status,*spi_packet_cnt);
	//DPRINTK("GPIOC_CONFIG=0x%x \n",__raw_readl(IMX6_GPCCON));
	//DPRINTK("GPIOC_PUD=0x%x \n",__raw_readl(IMX6_GPCPUD));

}


//控制使能信号css 0:enable 1:disenable
static void SPI_CSControl(int cs)
{
	printk("+++SPI_CSControl+++");
	volatile u32 tmp;
	if(cs)
	{
		*spi_cs_reg |= (1<<0);
		gpio_direction_output(SABRESD_ECSPI2_CS0, 1);
		gpio_set_value(SABRESD_ECSPI2_CS0, 1);
		*spi_cs_reg |= (1<<0);
	}
	else
	{
		gpio_direction_output(SABRESD_ECSPI2_CS0, 1);
			gpio_set_value(SABRESD_ECSPI2_CS0, 0);
		*spi_cs_reg &= ~(1<<0);
	}
	//DPRINTK("pclk*******=0x%x\n", *spi_cs_reg);
}


//每次发完数据后都要复位
static void gsensor_reset()
{
	*spi_ch_cfg |= IMX6_SPI_CH_SW_RST;
	//delayLoop(0x1ffff);//:::::::::::::
	udelay(100);
	printk("++++ gsensor_reset+++");
	*spi_ch_cfg &= ~IMX6_SPI_CH_SW_RST;

	*spi_mode_cfg &= ~(0x3<<1);//poll方式 以字节形式发送接受

	*spi_ch_cfg &= ~(0x3);
	//delayLoop(0x1ffff);
	udelay(100);
	DPRINTK("spi_ch_cfg=0x%x, spi_clk_cfg=0x%x, spi_mode_cfg=0x%x, spi_cs_reg=0x%x ,spi_int_en = 0x%x ,spi_status = 0x%x\n", *spi_ch_cfg, *spi_clk_cfg, *spi_mode_cfg,*spi_cs_reg,*spi_int_en,*spi_status);

}

//发送地址和所设置数据
static void gsensor_Send(unsigned char addr, unsigned char dat)
{
	int i;
	unsigned char temp;
	printk("+++ gsensor_Send+++");
	//gsensor_reset();
	gsensor_reset();

	SPI_CSControl(0);
	udelay(100);
	*spi_ch_cfg |= 0x3;

	//等待发送寄存器准备好
	while(!((*spi_status>>25)&0x1));

	*spi_tx_dat = addr;

	DPRINTK("spi_ch_cfg=0x%x, spi_clk_cfg=0x%x, spi_mode_cfg=0x%x, spi_cs_reg=0x%x ,spi_int_en = 0x%x ,spi_status = 0x%x\n", *spi_ch_cfg, *spi_clk_cfg, *spi_mode_cfg,*spi_cs_reg,*spi_int_en,*spi_status);
	while(!((*spi_status>>25)&0x1));

	*spi_tx_dat = dat;

	udelay(100);


    	SPI_CSControl(1);
}

//读取特定地址下的数据
static unsigned char gsensor_Recv(unsigned char addr)//这可能有问题
{
	unsigned char ch = 0;
	int i;
	printk("+++gsensor_Recv+++");
	gsensor_reset();

	SPI_CSControl(0);//拉低CSB信号线

	udelay(100);
	*spi_ch_cfg |= 0x3;

    	while(!((*spi_status>>25)&0x1))
    	printk("*spi_status %x \n",*spi_status);


	*spi_tx_dat = addr|0x80;

	ch= (*spi_rx_dat);
	DPRINTK("smb380 gggggggggg %x\n",ch);
	udelay(100);

	while(!((*spi_status>>25)&0x1));

	*spi_tx_dat = addr|0x80;

	ch= (*spi_rx_dat);
	DPRINTK("smb380 gffffffffff %x\n",ch);
	udelay(100);


	while(!((*spi_status>>25)&0x1));


	*spi_tx_dat = addr|0x80;
DPRINTK("calvin added:point1\n");
	while(!((*spi_status>>15)&0x7f));
//	udelay(1000);
DPRINTK("calvin added:point2\n");
	ch= (*spi_rx_dat);
	DPRINTK("smb380 gffffffffff %x\n",ch);
	udelay(100);

DPRINTK("point3\n");
	udelay(100);

        SPI_CSControl(1);

	 return ch;

}


//SPI方式初始化SMB380
static char InitSMB380(void)
{
	unsigned char temp;
	int i;

	printk("+++smb380 init start+++\n");


	gsensor_Send(OPER_REG,0xa2);
	gsensor_Send(OPER_SET,0);
	gsensor_Send(OPER_SET,(eight_G<<3)|bandWidth_25);
	gsensor_Send(INTERRUPT_SET,0x00);//disenable interrupt 0b
	temp = gsensor_Recv(SMB380_CHIP_ID);

	for(i=0;i<=0x15;i++)
	{
		temp = gsensor_Recv(i);
		DPRINTK("temp addr %x:%x\n",i,temp);
	}

	temp = gsensor_Recv(SMB380_CHIP_ID);

	DPRINTK("SMB380_CHIP_ID == %d\n", temp);
	return ((temp==2)?0:1);

}

//读取加速度的值
static unsigned short ReadAcc(unsigned char channel)
{
	short temp_h=0,temp_l=0;
	printk("+++ReadAcc+++");
	switch(channel)
	{
		case Axis_X:
			{
				while(!(gsensor_Recv(ACC_X_LSB)&0x1));
				temp_l = gsensor_Recv(ACC_X_LSB);//一定要先读低电平
				temp_h = gsensor_Recv(ACC_X_MSB);
				break;
			}
		case Axis_Y:
			{
				while(!(gsensor_Recv(ACC_Y_LSB)&0x1));
				temp_l = gsensor_Recv(ACC_Y_LSB);
				temp_h = gsensor_Recv(ACC_Y_MSB);
				break;
			}
		case Axis_Z:
			{
				while(!(gsensor_Recv(ACC_Z_LSB)&0x1));
				temp_l = gsensor_Recv(ACC_Z_LSB);
				temp_h = gsensor_Recv(ACC_Z_MSB);
				break;
			}
		default:
				break;
	}
	temp_h <<= 2;
	temp_l = (temp_l>>6)&0x03;
	temp_h |= temp_l;
	return temp_h;
}





static long gsensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)//struct inode *inode,
{

	char ret = 0;
	char i;
	unsigned char axis;
	unsigned int temp;
	float acceler;
	printk("++++ gsensor_ioctl+++");

	switch(cmd)
	{
		case SPI_CHANNEL_SET:
				channel = arg&0x1;
				gsensor_init(ucSpiBaud);
				i = InitSMB380();
				if(i != 0)
				{
					printk("[open ] SMB380 is error\n");
					return -1;
				}
				break;
		case SPI_BAUDRATE_SET:
				gsensor_init(ucSpiBaud);
				i = InitSMB380();
				if(i != 0)
				{
					printk("[open ] SMB380 is error\n");
					return -1;
				}
				break;
		case X_AXIS_READ:
				axis = (unsigned char)(arg&0xff);
				temp = ReadAcc(axis);
				printk("axis X :%d\n",temp);
				if(temp>0x400)
				{
					printk("[READ] SMB380 is error\n");
					return -1;
				}
				break;
		case Y_AXIS_READ:
				axis = (unsigned char)(arg&0xff);
				temp = ReadAcc(axis);
				printk("axis Y :%d\n",temp);
				if(temp>0x400)
				{
					printk("[READ] SMB380 is error\n");
					return -1;
				}
				break;
		case Z_AXIS_READ:
				axis = (unsigned char)(arg&0xff);
				temp = ReadAcc(axis);
				printk("axis Z :%d\n",temp);
				if(temp>0x400)
				{
					printk("[READ] SMB380 is error\n");
					return -1;
				}
				break;
		case SMB380_INTERRUPTMODE:
				break;
		case UPDATE_SMB380_IMG:
				break;
#if 1
		case IOCTL_S_POLL_INTERVAL:
			if(arg == 0)
			{
				del_timer(&(timer));
			}
			else
			{
				if(g_poll_interval == 0)
				{
					init_timer(&timer);
					timer.data= 0;
					timer.expires=jiffies +(58*HZ);
					timer.function=g_sensor_input_dev_poll;
					add_timer(&timer);
				}
				mod_timer(&(timer), jiffies + arg*(HZ/10));
			}
			g_poll_interval = arg;
			break;
#endif
		default:
				break;
	}

	return 0;
}


//when open beep device, this function will be called
static int gsensor_open(struct inode *inode, struct file *file)
{
	char i;
	printk("++++gsensor open++++");
	i = InitSMB380();
	if(i!=0) //是否正确打开了SMB380
	{
		printk("[open ] SMB380 is error\n");
		return -1;
	}

	return 0;
}

//只能以不译码的方式去写数字
static int gsensor_write(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	unsigned long err;
	;
	return 0;
}

static int gsensor_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	unsigned long err;
	float acceler;
	int i;
	printk("+++gsensor_read+++");
	u32 values[4];

	values[0] = ReadAcc(Axis_X);//X 分量

	values[1] = ReadAcc(Axis_Y);//Y 分量

	values[2] = ReadAcc(Axis_Z);//Z 分量

	values[3] = (gsensor_Recv(OPER_SET)>>3)&0x3;

	DPRINTK("X_value=%d\n",values[0]);
	DPRINTK("Y_value=%d\n",values[1]);
	DPRINTK("Z_value=%d\n",values[2]);
	DPRINTK("Range=%d\n",values[3]);

	err = copy_to_user(buff, (const void *)values, min(sizeof(values), count));

	return err ? -EFAULT : min(sizeof(values), count);

}

/*关闭设备的接口*/
static int gsensor_close(struct inode *inode, struct file *file)
{
	return 0;
}

#ifdef USER_POLL_READ
static void g_sensor_input_dev_poll(struct input_polled_dev *dev)
#else
static void g_sensor_input_dev_poll(int para)
#endif
{
	 int i;
	 s16 x, y, z;
	u32 values[4];
	printk("++++++++g_sensor_input_dev_poll+++++++");
	values[0] = ReadAcc(Axis_X);//X 分量

	values[1] = ReadAcc(Axis_Y);//Y 分量

	values[2] = ReadAcc(Axis_Z);//Z 分量

	values[3] = (gsensor_Recv(OPER_SET)>>3)&0x3;

	 /* convert signed 10bits to signed 16bits */
	 x = (short)(values[0] << 6) >> 6;
	 y = (short)(values[1] << 6) >> 6;
	 z = (short)(values[2] << 6) >> 6;
#ifdef USER_POLL_READ
	 input_report_abs(g_sensor_input_dev->input, ABS_X, x);
	 input_report_abs(g_sensor_input_dev->input, ABS_Y, y);
	 input_report_abs(g_sensor_input_dev->input, ABS_Z, z);
	 input_sync(g_sensor_input_dev->input);

#else
	 input_report_abs(g_sensor_input_dev, ABS_X, x);
	 input_report_abs(g_sensor_input_dev, ABS_Y, y);
	 input_report_abs(g_sensor_input_dev, ABS_Z, z);
	 input_sync(g_sensor_input_dev);
//	mod_timer(&(timer), jiffies + g_poll_interval*(HZ/10));
	mod_timer(&(timer), jiffies + g_poll_interval*(HZ/100));

#endif

//	 printk("dbg:report_abs()/x=%d,y=%d,z=%d\n",x,y,z);
}


/*接口注册*/
static struct file_operations gsensor_spi_fops=
{
	.owner		=	THIS_MODULE,
	.unlocked_ioctl	=	gsensor_ioctl,
	.open 		= 	gsensor_open,
	.release 	= 	gsensor_close,
	.write		= 	gsensor_write,
	.read		=	gsensor_read,
};

/*设备结构的设置*/
static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &gsensor_spi_fops,
};




static int gsensor_suspend(struct platform_device *dev, pm_message_t state)
{

	printk("+++++suspend call+++++++\n");
#if 0
	del_timer(&(timer));
//	clk_disable(ts_clock);
#endif
	return 0;
}
static int gsensor_resume(struct platform_device *pdev)
{
	printk("++++++++gsensor resume call+++++++++\n");
	gsensor_init(ucSpiBaud);
	InitSMB380();
#if 0
	init_timer(&timer);
	timer.data= 0;
	if(g_poll_interval == 0)
		timer.expires=jiffies +(58*HZ);
	else
		timer.expires = jiffies + g_poll_interval*(HZ/10);
	timer.function=g_sensor_input_dev_poll;
	add_timer(&timer);
#endif
//	clk_enable(ts_clock);

	return 0;
}

/*设备初始化函数*/
static int __init gsensor_probe(struct platform_device *pdev)
{
	int ret;
	unsigned long j,i;
	printk("+++++gsensor_probe++++");
	unsigned char temp;
	struct input_dev *idev;
	gsensor_init(ucSpiBaud);
	temp = InitSMB380();
	g_poll_interval = 2;
	if(temp!=0)
	{
		printk(KERN_INFO "[init] SMB380 is error!\n");
		return -1;
	}

	ret = misc_register(&misc);

	if(ret!=0)
	{
		printk("init smb380 unsuccessfully!\n");
		return -1;
	}
	printk("init smb380 successfully!\n");
#ifdef USER_POLL_READ
 	/*input poll device register */
	g_sensor_input_dev = input_allocate_polled_device();
#else
	g_sensor_input_dev = input_allocate_device();
#endif
	if (!g_sensor_input_dev)
	 {
  		printk("alloc  device failed!\n");
	 }
	else{
#ifdef USER_POLL_READ
		g_sensor_input_dev->poll = g_sensor_input_dev_poll;
		g_sensor_input_dev->poll_interval = POLL_INTERVAL;
		idev = g_sensor_input_dev->input;
#else
		init_timer(&timer);
		timer.data= 0;
		timer.expires=jiffies +(10*HZ);
		timer.function=g_sensor_input_dev_poll;
		add_timer(&timer);
		idev = g_sensor_input_dev;
#endif
 		idev->name = "Android Gravity Sensor";
		idev->evbit[0] = BIT_MASK(EV_ABS);
		idev->id.bustype=BUS_SPI;
		idev->id.vendor=0x0288;
		idev->id.product=0x0128;
		idev->id.version=0x0052;
		__set_bit(EV_ABS,idev->evbit);
		__set_bit(ABS_PRESSURE,idev->absbit);
		__set_bit(EV_SYN,idev->evbit);
		 //set absolute  coordinate maximum range (-512,512) ?
		 input_set_abs_params(idev, ABS_X, -4096, 4096, /*INPUT_FUZZ*/0, /*INPUT_FLAT*/0);
		 input_set_abs_params(idev, ABS_Y, -4096, 4096, /*INPUT_FUZZ*/0, /*INPUT_FLAT*/0);
		 input_set_abs_params(idev, ABS_Z, -4096, 4096, /*INPUT_FUZZ*/0, /*INPUT_FLAT*/0);
#ifdef USER_POLL_READ
		 ret = input_register_polled_device(g_sensor_input_dev);
		 if (ret) {
		  printk("register poll device failed!\n");
		 }
#else
		ret = input_register_device(idev);
		if (ret < 0) {
			input_free_device(idev);
			return ret;
		}
#endif
	}
	return 0;
}

/*卸载函数*/
static int gsensor_remove(struct platform_device *dev)
{

	misc_deregister(&misc);
	del_timer(&timer);
	printk("EmbedSky-spi module exit !\n");
}

static struct platform_driver imx6_gsensor_driver = {
       .probe          = gsensor_probe,
       .remove         = gsensor_remove,
       .suspend        = gsensor_suspend,
       .resume         = gsensor_resume,
       .driver		= {
		.owner	= THIS_MODULE,
		.name	= "Acceleration_sensor",
	},
};



static int __init imx6_gsensor_init(void)
{
	DPRINTK("Enter imx6_gsensor_init\n");
	return platform_driver_register(&imx6_gsensor_driver);
}

static void __exit imx6_gsensor_exit(void)
{
	printk("Enter imx6_gsensor_exit\n");

	platform_driver_unregister(&imx6_gsensor_driver);
}

module_init(imx6_gsensor_init);
module_exit(imx6_gsensor_exit);

MODULE_AUTHOR("Samsung AP");
MODULE_DESCRIPTION("S5P touchscreen driver");
MODULE_LICENSE("GPL");

