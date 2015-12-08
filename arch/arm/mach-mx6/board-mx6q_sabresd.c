/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/power/sabresd_battery.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/gpio.h>
#include <sound/wm8960.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_sabresd.h"
//#include "board-mx6dl_sabresd.h"
#include <mach/imx_rfkill.h>
#include <linux/log_msg.h>
#include "tq_e9.h"
#define SABRESD_BT_RESET        IMX_GPIO_NR(1, 2)
#define SABRESD_MICROPHONE_DET  IMX_GPIO_NR(3, 9)
#define SABRESD_RGMII_INT       IMX_GPIO_NR(1, 26)

#define SABRESD_SD3_CD          IMX_GPIO_NR(1, 30)
#define SABRESD_SD3_WP          IMX_GPIO_NR(2, 29)

#define SABRESD_SD2_CD          IMX_GPIO_NR(1, 4)
#define SABRESD_SD2_WP          IMX_GPIO_NR(1, 2)
#define SABRESD_SD1_CMD_3G          IMX_GPIO_NR(1,18)

#define SABRESD_CSI0_PWN_tmp    IMX_GPIO_NR(5, 20)
#define SABRESD_CSI0_PWN        IMX_GPIO_NR(7, 13)
#define SABRESD_CSI0_RST        IMX_GPIO_NR(7, 12)
#define SABRESD_GPIO_TPINT      IMX_GPIO_NR(1, 9)
#define SABRESD_GPIO_TPPWN      IMX_GPIO_NR(1, 16)
#define SABRESD_ACCL_INT        IMX_GPIO_NR(2, 18)
#define SABRESD_MIPICSI_PWN     IMX_GPIO_NR(2, 19)
#define SABRESD_MIPICSI_RST     IMX_GPIO_NR(2, 20)

#define SABRESD_GPIO_VOL_UP     IMX_GPIO_NR(4, 8)   //volume ++
#define SABRESD_GPIO_VOL_DOWN   IMX_GPIO_NR(4, 9)   //volume --
#define SABRESD_GPIO_POWER      IMX_GPIO_NR(4, 5)   //power
#define BOARD_POWER_CTRL        IMX_GPIO_NR(6, 31)  //power ctrl

#define SABERSD_GPIO_DS18B20    IMX_GPIO_NR(2, 29)
#define HS0038_GPIO_IRQ         IMX_GPIO_NR(1, 6)
#define SABRESD_HEADPHONE_DET   IMX_GPIO_NR(3, 31)
#define GPIO_I2C5_IRQ		IMX_GPIO_NR(1, 28)

#define SABRESD_CHARGE_DOK_B    IMX_GPIO_NR(2, 24)
#define SABRESD_SENSOR_EN       IMX_GPIO_NR(2, 31)

#define SABRESD_DISP0_RST_B     IMX_GPIO_NR(3, 8)
#define SABRESD_ALS_INT         IMX_GPIO_NR(3, 9)
#define SABRESD_CHARGE_CHG_2_B  IMX_GPIO_NR(3, 13)
#define SABRESD_CHARGE_FLT_2_B  IMX_GPIO_NR(3, 14)
#define SABRESD_BAR0_INT        IMX_GPIO_NR(3, 15)
#define SABRESD_eCOMPASS_INT    IMX_GPIO_NR(3, 16)
#define SABRESD_PCIE_PWR_EN     IMX_GPIO_NR(3, 19)
#define SABRESD_USB_OTG_PWR     IMX_GPIO_NR(7, 1)
#define SABRESD_USB_H1_PWR      IMX_GPIO_NR(2, 29)
#define SABRESD_CHARGE_CHG_1_B  IMX_GPIO_NR(3, 23)
#define SABRESD_TS_INT          IMX_GPIO_NR(3, 26)

#define SABRESD_CAN1_STBY       IMX_GPIO_NR(4, 7)
#define SABRESD_ECSPI1_CS0      IMX_GPIO_NR(4, 9)
#define SABRESD_CODEC_PWR_EN    IMX_GPIO_NR(4, 10)
#define SABRESD_HDMI_CEC_IN     IMX_GPIO_NR(4, 11)
#define SABRESD_PCIE_DIS_B      IMX_GPIO_NR(4, 14)

#define SABRESD_DI0_D0_CS   IMX_GPIO_NR(5, 0)
#define SABRESD_CHARGE_FLT_1_B  IMX_GPIO_NR(5, 2)
#define SABRESD_PCIE_WAKE_B 	IMX_GPIO_NR(5, 21)

#define SABRESD_CAP_TCH_INT1    IMX_GPIO_NR(6, 7)
#define SABRESD_CAP_TCH_INT0    IMX_GPIO_NR(6, 8)
#define SABRESD_DISP_RST_B  	IMX_GPIO_NR(6, 11)
#define SABRESD_DISP_PWR_EN 	IMX_GPIO_NR(6, 14)
#define SABRESD_CABC_EN0    	IMX_GPIO_NR(6, 15)
#define SABRESD_CABC_EN1    	IMX_GPIO_NR(6, 16)
#define SABRESD_AUX_3V15_EN 	IMX_GPIO_NR(6, 9)
#define SABRESD_DISP0_WR_REVB   IMX_GPIO_NR(6, 9)
#define SABRESD_AUX_5V_EN   	IMX_GPIO_NR(6, 10)
#define SABRESD_DI1_D0_CS   	IMX_GPIO_NR(6, 31)

#define SABRESD_PCIE_RST_B_REVB IMX_GPIO_NR(7, 12)
#define SABRESD_PMIC_INT_B  	IMX_GPIO_NR(7, 13)
#define SABRESD_PFUZE_INT   	IMX_GPIO_NR(7, 13)

#define SABRESD_CHARGE_NOW  	IMX_GPIO_NR(1, 2)
#define SABRESD_CHARGE_DONE 	IMX_GPIO_NR(1, 1)

#define GPIO_I2C5_SDA			IMX_GPIO_NR(3, 18)						
#define GPIO_I2C5_SCK			IMX_GPIO_NR(3, 17)


#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ        	IMX_GPIO_NR(1, 26)
#define IOMUX_OBSRV_MUX1_OFFSET 0x3c
#define OBSRV_MUX1_MASK         0x3f
#define OBSRV_MUX1_ENET_IRQ     0x9
#endif



static struct clk *clko;
static int caam_enabled;
static int uart5_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;


static const struct esdhc_platform_data mx6q_sabresd_sd2_data __initconst =
{
	.cd_gpio = SABRESD_SD2_CD,
//	.wp_gpio = SABRESD_SD2_WP,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

static const struct esdhc_platform_data mx6q_sabresd_sd3_data __initconst =
{
	//  .cd_gpio = SABRESD_SD3_CD,
	//  .wp_gpio = SABRESD_SD3_WP,
//	.always_present = 1,
	.keep_power_at_suspend = 1,
	//  .support_8bit = 0,
	//  .delay_line = 0,
//	.cd_type = ESDHC_CD_PERMANENT,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_sabresd_sd4_data __initconst =
{
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
		mx6q_sabresd_anatop_thermal_data __initconst =
{
	.name = "anatop_thermal",
};

static const struct imxuart_platform_data mx6q_sd_uart5_data __initconst =
{
	.flags      = IMXUART_HAVE_RTSCTS,
	.dma_req_rx = MX6Q_DMA_REQ_UART5_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART5_TX,
};

static inline void mx6q_sabresd_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(3, NULL);
	//  imx6q_add_imx_uart(4, NULL);
}

static struct fec_platform_data fec_data __initdata =
{
	.init = mx6q_sabresd_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static int mx6q_sabresd_spi_cs[] =
{
	SABRESD_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_sabresd_spi_data __initconst =
{
	.chipselect     = mx6q_sabresd_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_sabresd_spi_cs),
};

static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata =
{
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_sabresd_audio_wm8960_device =
{
	.name = "imx-wm8960",
};

static struct mxc_audio_platform_data wm8960_data;

static int wm8960_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);

	return 0;
}

static int mxc_wm8960_init(void)
{
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko))
	{
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/* both audio codec and comera use CLKO clk*/
	rate = clk_round_rate(clko, 24000000);
	clk_set_rate(clko, rate);

	wm8960_data.sysclk = rate;

	return 0;
}

static struct wm8960_data wm8960_config_data =
{
	.gpio_init = {
		[2] = WM8960_GPIO_FN_DMICCLK,
		[4] = 0x8000 | WM8960_GPIO_FN_DMICDAT,
	},
};

static struct mxc_audio_platform_data wm8960_data =
{
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = SABRESD_HEADPHONE_DET,
	.hp_active_low = 1,
	.mic_gpio = SABRESD_MICROPHONE_DET,
	.mic_active_low = 1,
	.init = mxc_wm8960_init,
	.clock_enable = wm8960_clk_enable,
};
//---------------------- hs0038-----------------------
static struct gpio_keys_button tq_hs0038[] =
{
	{
		.gpio       = HS0038_GPIO_IRQ,/* 4(- + backspace enter) */
		.desc       = "hs0038",
	},
};

static struct gpio_keys_platform_data tq_hs0038_data =
{
	.buttons    = tq_hs0038,
	.nbuttons   = ARRAY_SIZE(tq_hs0038),
};

static struct platform_device tq_hs0038_device =
{
	.name       = "hs0038",
	.id     = -1,
	.dev        = {
		.platform_data  = &tq_hs0038_data,
	}
};
//----------------------tsc2007----------------------

#include <linux/i2c/tsc2007.h>
#define SABRESD_GPIO_TSC2007IRQ	IMX_GPIO_NR(1, 29)
struct tsc2007_platform_data  tq_tsc2007_data = {
	.model = 2007,
	.x_plate_ohms = 180,
	.fuzzx = 0,
	.fuzzy = 0,
	.fuzzz = 0,
	.init_platform_hw = tq_tsc_init_hw,
	.exit_platform_hw = tq_tsc_exit_hw,
	.get_pendown_state = tq_tsc_state_hw,
	.irq_pin = SABRESD_GPIO_TSC2007IRQ,
};

static struct regulator_consumer_supply sabresd_vwm8960_consumers[] =
{
	REGULATOR_SUPPLY("SPKVDD1", "1-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "1-001a"),
};

static struct regulator_init_data sabresd_vwm8960_init =
{
	.constraints = {
		.name = "SPKVDD",
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vwm8960_consumers),
	.consumer_supplies = sabresd_vwm8960_consumers,
};

static struct fixed_voltage_config sabresd_vwm8960_reg_config =
{
	.supply_name    = "SPKVDD",
	.microvolts     = 4200000,
	.gpio           = SABRESD_CODEC_PWR_EN,
	.enable_high    = 1,
	.enabled_at_boot = 1,
	.init_data      = &sabresd_vwm8960_init,
};

static struct platform_device sabresd_vwm8960_reg_devices =
{
	.name   = "reg-fixed-voltage",
	.id     = 4,
	.dev    = {
		.platform_data = &sabresd_vwm8960_reg_config,
	},
};

static void mx6q_board_powerctrl(int onoff)
{
	int rtn;
	rtn = gpio_request(BOARD_POWER_CTRL, "mx6_power_ctrl");
	if (rtn < 0)
	{

		printk("can't get gpio for power ctrl\n");
		return ;
	}
	if (onoff != 0)
		gpio_direction_output(BOARD_POWER_CTRL, 1);
	else
		gpio_direction_output(BOARD_POWER_CTRL, 0);
	gpio_free(BOARD_POWER_CTRL);
}

static void mx6_poweroff(void)
{
	mx6q_board_powerctrl(0);
}

static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown != 0)     //off
	{
		gpio_set_value(SABRESD_CSI0_RST, 0);
		msleep(10);
		gpio_set_value(SABRESD_CSI0_PWN, 1);                //cam0 off
		gpio_set_value(SABRESD_CSI0_PWN_tmp, 1);                //cam0 off
	}
	else                                                    //cam1 not refrect
	{
		gpio_set_value(SABRESD_CSI0_RST, 1);
		msleep(10);
		gpio_set_value(SABRESD_CSI0_PWN, 0);                //cam0 on
		gpio_set_value(SABRESD_CSI0_PWN_tmp, 0);                //cam0 on
	}

	msleep(10);
}

static void mx6q_csi0_io_init(void)
{

	mxc_iomux_v3_setup_multiple_pads(mx6q_e9_csi0_sensor_pads, \
	                                 ARRAY_SIZE(mx6q_e9_csi0_sensor_pads));

	/* Camera reset */
	gpio_request(SABRESD_CSI0_RST, "cam-reset");
	gpio_direction_output(SABRESD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(SABRESD_CSI0_PWN, "cam-pwdn");
	gpio_request(SABRESD_CSI0_PWN_tmp, "cam-pwdn");
	gpio_direction_output(SABRESD_CSI0_PWN, 1);
	gpio_direction_output(SABRESD_CSI0_PWN_tmp, 1);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_PWN, 0);
	gpio_set_value(SABRESD_CSI0_PWN_tmp, 0);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(SABRESD_CSI0_RST, 1);
	msleep(5);
	gpio_set_value(SABRESD_CSI0_PWN, 1);
	gpio_set_value(SABRESD_CSI0_PWN_tmp, 1);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 *
	 * For MX6DL:
	 * GPR13 bit 0-2 IPU_CSI0_MUX
	 *   000 MIPI_CSI0
	 *   100 IPU CSI0
	 */
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}
static struct fsl_mxc_camera_platform_data camera_data_ov3640 =
{
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

/*
static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
    .mclk = 24000000,
    .mclk_source = 0,
    .csi = 1,
    .io_init = mx6q_mipi_sensor_io_init,
    .pwdn = mx6q_mipi_powerdown,
};
*/
#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)


static void tq_capts_poweron(int onoff)
{
	int err;
	err = gpio_request(SABRESD_GPIO_TPPWN, "cap_ts_power\n");
	if (err < 0)
	{
		printk("Error: can't request power gpio %d!!\n", SABRESD_GPIO_TPPWN);
		return ;
	}
	if (onoff != 0)
		gpio_direction_output(SABRESD_GPIO_TPPWN, 1);
	else
		gpio_direction_output(SABRESD_GPIO_TPPWN, 0);
	gpio_free(SABRESD_GPIO_TPPWN);
	msleep(20);
}
/*
*/
struct tq_captp_platform_data gt811_platform_data = {
    .irq_pin = SABRESD_GPIO_TPINT,
    .power_on = tq_capts_poweron,
};

#include <linux/i2c-gpio.h>
static struct i2c_gpio_platform_data i2c5_platdata = {
	.sda_pin                = GPIO_I2C5_SDA,
	.scl_pin                = GPIO_I2C5_SCK,
	.udelay                 = 5,	// freq = 500/udelay
	.sda_is_open_drain      = 0,
	.scl_is_open_drain      = 0,
	.scl_is_output_only     = 0,
};

static struct platform_device   mx6_i2c5_device = {
	.name                   = "i2c-gpio",
	.id                     = 5,
	.dev.platform_data      = &i2c5_platdata,
};

static struct i2c_board_info mxc_i2c5_gpio_board_info[] __initdata =
{
	{
		I2C_BOARD_INFO("sc16is752", 0x48),
		.irq = gpio_to_irq(GPIO_I2C5_IRQ),
	},
};

/*
*/
#ifdef CONFIG_TOUCHSCREEN_FT5_I2C
struct tq_captp_platform_data ft5x06_platform_data = {
    .irq_pin = SABRESD_GPIO_TPINT,
    .power_on = tq_capts_poweron,
};
#endif
void ch7034b_rest(void)
{
	;
}

static struct fsl_mxc_lcd_platform_data ch7034b_platform_data =
{
	.reset = ch7034b_rest,
};

static struct imxi2c_platform_data mx6q_sabresd_i2c_data =
{
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata =
{
	{
		I2C_BOARD_INFO("ch7034", 0x75),
		.platform_data = (void *) &ch7034b_platform_data,
	},
#if 0
	{
		I2C_BOARD_INFO("cap_ts", 0x5d),
		.platform_data = (void*)&gt811_platform_data,
	},
#else
	{
		I2C_BOARD_INFO("Goodix-TS", 0x5d),//gt9xx
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_FT5_I2C)
	{
	   I2C_BOARD_INFO("ft5x06_ts", 0x38),
	   .platform_data =  (void*)&ft5x06_platform_data,
	},
#endif
	{
		I2C_BOARD_INFO("tsc2007",(0x90>>1)),
		.platform_data = &tq_tsc2007_data,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata =
{
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
		.platform_data = (void *) &wm8960_config_data,
	},
	{
		I2C_BOARD_INFO("ov3640", 0x3c),//0x78 0x3c
		.platform_data = (void *) &camera_data_ov3640,
	},

};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata =
{
	{
		I2C_BOARD_INFO("mxc_ldb_i2c", 0x50),
		.platform_data = (void *)1, /* lvds port1 */
	},
};

static void imx6q_sabresd_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_OTG_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_OTG_PWR, 0);
}

static void imx6q_sabresd_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(SABRESD_USB_H1_PWR, 1);
	else
		gpio_set_value(SABRESD_USB_H1_PWR, 0);
}

static void __init imx6q_sabresd_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(SABRESD_USB_OTG_PWR, "usb-pwr");
	if (ret)
	{
		pr_err("failed to get GPIO SABRESD_USB_OTG_PWR: %d\n",
		       ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(SABRESD_USB_H1_PWR, "usb-h1-pwr");
	if (ret)
	{
		pr_err("failed to get GPIO SABRESD_USB_H1_PWR: %d\n",
		       ret);
		return;
	}
	gpio_direction_output(SABRESD_USB_H1_PWR, 0);
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_sabresd_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_sabresd_host1_vbus);

}


#ifdef CONFIG_SATA_AHCI_PLATFORM

static struct ahci_platform_data mx6q_sabresd_sata_data =
{
	.init = mx6q_sabresd_sata_init,
	.exit = mx6q_sabresd_sata_exit,
};
#endif

static void mx6q_sabresd_flexcan0_switch(int enable)
{
	if (enable)
	{
		gpio_set_value(SABRESD_CAN1_STBY, 1);
	}
	else
	{
		gpio_set_value(SABRESD_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
		mx6q_sabresd_flexcan0_pdata __initconst =
{
	.transceiver_switch = mx6q_sabresd_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata =
{
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct imx_asrc_platform_data imx_asrc_data =
{
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
	gpio_set_value(SABRESD_DISP_PWR_EN, 1);
	gpio_set_value(SABRESD_DISP_RST_B, 1);
	udelay(10);
	gpio_set_value(SABRESD_DISP_RST_B, 0);
	udelay(50);
	gpio_set_value(SABRESD_DISP_RST_B, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata =
{
	.ipu_id     = 0,
	.disp_id    = 1,
	.lcd_panel  = "TRULY-WVGA",
	.reset      = mx6_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data sabresd_fb_data[] =
{
	{
		.disp_dev = "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str = "LDB-XGA",
		.default_bpp = 16,
		.int_clk = false,
		.late_init = false,
	},
#if 0
	{ /*fb0*/
		.disp_dev = "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str = "TQ_ldb_1024768",
		.default_bpp = 16,
		.int_clk = false,
		.late_init = false,
	},
#endif
	{
		.disp_dev = "hdmi",
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "1920x1080M@60",
		.default_bpp = 32,
		.int_clk = false,
		.late_init = false,
	}, 
	{
		.disp_dev = "lcd",
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "TQ-TN92",
		.default_bpp = 24,
		.int_clk = false,
		.late_init = false,
	}, 
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0))
	{
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0))
	{
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2 * ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sabresd board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */
// if use 6dl pin need change
static void hdmi_enable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_hdmi_ddc_pads,		                                 ARRAY_SIZE(mx6q_sabresd_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_i2c2_pads,
		                                 ARRAY_SIZE(mx6q_sabresd_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data =
{
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data =
{
	.ipu_id = 1,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data =
{
	.ipu_id = 0,
	.disp_id = 0,
	//  .default_ifmt = IPU_PIX_FMT_RGB565,
	.default_ifmt = IPU_PIX_FMT_RGB24,
};

static struct fsl_mxc_ldb_platform_data ldb_data =
{
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 0,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] =
{
	{
		.rev = 4,
		.csi_clk[0] = "clko_clk",
		.bypass_reset = false,
	}, {
		.rev = 4,
		.csi_clk[0] = "clko_clk",
		.bypass_reset = false,
	},
};

static struct ion_platform_data imx_ion_data =
{
	.nr = 1,
	.heaps = {
		{
			.id = 0,
			.type = ION_HEAP_TYPE_CARVEOUT,
			.name = "vpu_ion",
			.size = SZ_16M,
			.cacheable = 1,
		},
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] =
{
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


static void sabresd_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 0);
}

static void sabresd_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	gpio_set_value(SABRESD_AUX_5V_EN, 1);
}
static const struct pm_platform_data mx6q_sabresd_pm_data __initconst =
{
	.name = "imx_pm",
	.suspend_enter = sabresd_suspend_enter,
	.suspend_exit = sabresd_suspend_exit,
};

static struct regulator_consumer_supply sabresd_vmmc_consumers[] =
{
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabresd_vmmc_init =
{
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vmmc_consumers),
	.consumer_supplies = sabresd_vmmc_consumers,
};

static struct fixed_voltage_config sabresd_vmmc_reg_config =
{
	.supply_name        = "vmmc",
	.microvolts     = 3300000,
	.gpio           = -1,
	.init_data      = &sabresd_vmmc_init,
};

static struct platform_device sabresd_vmmc_reg_devices =
{
	.name   = "reg-fixed-voltage",
	.id = 3,
	.dev    = {
		.platform_data = &sabresd_vmmc_reg_config,
	},
};
#if CONFIG_SND_SOC_IMX_WM8960
static int __init imx6q_init_audio(void)
{

	platform_device_register(&sabresd_vwm8960_reg_devices);
	mxc_register_device(&mx6_sabresd_audio_wm8960_device,
	                    &wm8960_data);
	imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

	mxc_wm8960_init();

	return 0;
}
#endif

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)   \
{                                   \
.gpio           = gpio_led,             \
.name           = name_led,             \
.active_low     = act_low,              \
.retain_state_suspended = state_suspend,            \
.default_state      = 0,                    \
.default_trigger    = "max8903-"trigger,        \
}

/* use to show a external power source is connected
 * GPIO_LED(SABRESD_CHARGE_DONE, "chg_detect", 0, 1, "ac-online"),
 */
static struct gpio_led imx6q_gpio_leds[] =
{
	GPIO_LED(SABRESD_CHARGE_NOW, "chg_now_led", 0, 1,
	"charger-charging"),
	/* For the latest B4 board, this GPIO_1 is connected to POR_B,
	which will reset the whole board if this pin's level is changed,
	so, for the latest board, we have to avoid using this pin as
	GPIO.
	    GPIO_LED(SABRESD_CHARGE_DONE, "chg_done_led", 0, 1,
	            "charger-full"),
	*/
};
static struct gpio_led_platform_data imx6q_gpio_leds_data =
{
	.leds       = imx6q_gpio_leds,
	.num_leds   = ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device =
{
	.name       = "leds-gpio",
	.id     = -1,
	.num_resources  = 0,
	.dev        = {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

/* For BT_PWD_L is conflict with charger's LED trigger gpio on sabresd_revC.
 * add mutual exclusion here to be decided which one to be used by board config
 */
static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)  \
{                               \
	.gpio       = gpio_num,             \
	.type       = EV_KEY,               \
	.code       = ev_code,              \
	.active_low = act_low,              \
	.desc       = "btn " descr,             \
	.wakeup     = wake,                 \
	.debounce_interval = debounce,              \
}


static struct gpio_keys_button sabresd_buttons[] =
{
	GPIO_BUTTON(SABRESD_GPIO_VOL_DOWN,  KEY_ENTER, 1, "enter",   0, 1),//KEY_HOME
	GPIO_BUTTON(SABRESD_GPIO_VOL_UP,    KEY_HOME,   1, "exit",     0, 1),
	GPIO_BUTTON(SABRESD_GPIO_POWER,     KEY_POWER,      1, "power",         1, 1),
};

static struct gpio_keys_platform_data sabresd_button_data = {
	.buttons	= sabresd_buttons,
	.nbuttons	= ARRAY_SIZE(sabresd_buttons),
};

static struct platform_device sabresd_button_device =
{
	.name       = "gpio-keys",
	.id     = -1,
	.num_resources  = 0,
};

static void __init imx6q_add_device_buttons(void)
{
	/* fix me */
	/* For new sabresd(RevB4 ane above) change the
	 * ONOFF key(SW1) design, the SW1 now connect
	 * to GPIO_3_29, it can be use as a general power
	 * key that Android reuired. But those old sabresd
	 * such as RevB or older could not support this
	 * change, so it needs a way to distinguish different
	 * boards. Before board id/rev are defined cleary,
	 * there is a simple way to achive this, that is using
	 * SOC revison to identify differnt board revison.
	 *
	 * With the new sabresd change and SW mapping the
	 * SW1 as power key, below function related to power
	 * key are OK on new sabresd board(B4 or above).
	 *  1 Act as power button to power on the device when device is power off
	 *  2 Act as power button to power on the device(need keep press SW1 >5s)
	 *  3 Act as power key to let device suspend/resume
	 *  4 Act screenshort(hold power key and volume down key for 2s)
	 */
	platform_device_add_data(&sabresd_button_device,
	                         &sabresd_button_data,
	                         sizeof(sabresd_button_data));

	platform_device_register(&sabresd_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data mx6_sabresd_pwm_backlight_data =
{
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
                                   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = sabresd_fb_data;

	for_each_tag(t, tags)
	{
		if (t->hdr.tag == ATAG_CMDLINE)
		{
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL)
			{
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
				        i < ARRAY_SIZE(sabresd_fb_data))
				{
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL)
			{
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL)
			{
				str += 8;
				pdata_fb[0].res_base[0] =
				    simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL)
			{
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static struct mipi_csi2_platform_data mipi_csi2_pdata =
{
	.ipu_id  = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38

static const struct imx_pcie_platform_data mx6_sabresd_pcie_data __initconst =
{
	.pcie_pwr_en    = SABRESD_PCIE_PWR_EN,
	.pcie_rst   = SABRESD_PCIE_RST_B_REVB,
	.pcie_wake_up   = SABRESD_PCIE_WAKE_B,
	.pcie_dis   = SABRESD_PCIE_DIS_B,
	.pcie_power_always_on = 1,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource =
{
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console =
{
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif


/*!
 * Board specific initialization.
 */
static void __init mx6_sabresd_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;

	mxc_iomux_v3_setup_multiple_pads(mx6q_sabresd_pads, \
	                                 ARRAY_SIZE(mx6q_sabresd_pads));
	mxc_iomux_v3_setup_multiple_pads(mx6q_e9_pads, \
	                                 ARRAY_SIZE(mx6q_e9_pads));

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif
	//  gpio_test(SABRESD_GPIO_TPINT);

	//  gp_reg_id = sabresd_dvfscore_data.reg_id;
	//  soc_reg_id = sabresd_dvfscore_data.soc_id;
	mx6q_sabresd_init_uart();
	imx6x_add_ram_console();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl())
	{
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 1;
		hdmi_core_data.ipu_id = 0;
		hdmi_core_data.disp_id = 0;
		mipi_dsi_pdata.ipu_id = 0;
		mipi_dsi_pdata.disp_id = 1;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q())
	{
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);
	}
	else
		for (i = 0; i < 2 && i < ARRAY_SIZE(sabresd_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabresd_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabresd_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabresd_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
	                        ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
	                        ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
	                        ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(SABRESD_PFUZE_INT, "pFUZE-int");
	if (ret)
	{
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	}
	else
	{
		gpio_direction_input(SABRESD_PFUZE_INT);
		mx6q_sabresd_init_pfuze100(SABRESD_PFUZE_INT);
	}
	/* SPI */
	//  imx6q_add_ecspi(0, &mx6q_sabresd_spi_data);

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabresd_anatop_thermal_data);
	imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
	                                   OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif

	imx6q_add_pm_imx(0, &mx6q_sabresd_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabresd_sd4_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sabresd_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabresd_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabresd_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q())
	{
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabresd_sata_data);
#else
		mx6q_sabresd_sata_init(NULL,
		                       (void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabresd_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/************/
	platform_device_register(&tq_hs0038_device);

	platform_device_register(&mx6_i2c5_device);
	i2c_register_board_info(5, mxc_i2c5_gpio_board_info,\
							ARRAY_SIZE(mxc_i2c5_gpio_board_info));
	/*
	 * Disable HannStar touch panel CABC function,
	 * this function turns the panel's backlight automatically
	 * according to the content shown on the panel which
	 * may cause annoying unstable backlight issue.
	 */
	gpio_request(SABRESD_CABC_EN0, "cabc-en0");
	gpio_direction_output(SABRESD_CABC_EN0, 0);
	gpio_request(SABRESD_CABC_EN1, "cabc-en1");
	gpio_direction_output(SABRESD_CABC_EN1, 0);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_sabresd_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	//  imx6q_add_dvfs_core(&sabresd_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
		              sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	imx6q_add_device_buttons();

	/* enable sensor 3v3 and 1v8 */
	gpio_request(SABRESD_SENSOR_EN, "sensor-en");
	gpio_direction_output(SABRESD_SENSOR_EN, 1);

	/* enable ecompass intr */
	gpio_request(SABRESD_eCOMPASS_INT, "ecompass-int");
	gpio_direction_input(SABRESD_eCOMPASS_INT);
	/* enable light sensor intr */
	gpio_request(SABRESD_ALS_INT, "als-int");
	gpio_direction_input(SABRESD_ALS_INT);

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	imx6q_add_flexcan0(&mx6q_sabresd_flexcan0_pdata);
	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent))
	{
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	/* Enable Aux_5V */
	gpio_request(SABRESD_AUX_5V_EN, "aux_5v_en");
	gpio_direction_output(SABRESD_AUX_5V_EN, 1);
//	gpio_set_value(SABRESD_AUX_5V_EN, 1);
   	/*3g */
	gpio_request(SABRESD_SD1_CMD_3G, "3g");
	gpio_direction_output(SABRESD_SD1_CMD_3G, 0);
	gpio_set_value(SABRESD_SD1_CMD_3G,0);
	/* Register charger chips */
	imx6q_add_busfreq();

	/* Add PCIe RC interface support
	 * uart5 has pin mux with pcie. or you will use uart5 or use pcie
	 */
	if (!uart5_enabled)
		imx6q_add_pcie(&mx6_sabresd_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	pm_power_off = mx6_poweroff;
}

extern void __iomem *twd_base;
static void __init mx6_sabresd_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabresd_timer =
{
	.init   = mx6_sabresd_timer_init,
};

static void __init mx6q_sabresd_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(sabresd_fb_data);
	if (fb_array_size > 0 && sabresd_fb_data[0].res_base[0] &&
	        sabresd_fb_data[0].res_size[0])
	{
		if (sabresd_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			       sabresd_fb_data[0].res_base[0]);
		memblock_reserve(sabresd_fb_data[0].res_base[0],
		                 sabresd_fb_data[0].res_size[0]);
		memblock_remove(sabresd_fb_data[0].res_base[0],
		                sabresd_fb_data[0].res_size[0]);
		sabresd_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (sabresd_fb_data[i].res_size[0])
		{
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(sabresd_fb_data[i].res_size[0],
			                           SZ_4K, SZ_2G);
			memblock_remove(phys, sabresd_fb_data[i].res_size[0]);
			sabresd_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size)
	{
		//----------------------alloc SZ_2G---------------------
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
		                           SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size)
	{
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_SABRESD data structure.
 */
MACHINE_START(MX6Q_SABRESD, "Freescale i.MX 6Quad/DualLite/Solo Sabre-SD Board")
/* Maintainer: Freescale Semiconductor, Inc. */
.boot_params = MX6_PHYS_OFFSET + 0x100,
.fixup = fixup_mxc_board,
.map_io = mx6_map_io,
.init_irq = mx6_init_irq,
.init_machine = mx6_sabresd_board_init,
.timer = &mx6_sabresd_timer,
.reserve = mx6q_sabresd_reserve,
MACHINE_END
