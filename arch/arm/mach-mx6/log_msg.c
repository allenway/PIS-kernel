/* linux/drivers/media/video/samsung/jpeg/jpg_msg.c
 *
 * Driver file for Samsung JPEG Encoder/Decoder
 *
 * Peter Oh, Copyright (c) 2009 Samsung Electronics
 * 	http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <stdarg.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/param.h>

#include <linux/sched.h>
#include <linux/wait.h>	//wait_queue
#include <linux/ioport.h>//ioremap,request_mem_region;

#include <linux/log_msg.h>




static const LOG_LEVEL log_level = LOG_DEBUG;//,LOG_NOTE;//

static const char *modulename = "ZQL";

static const char *level_str[] = {"ERROR", "WARNING", "NOTE","DEBUG"};


void _log_msg(LOG_LEVEL level, const char *func_name, const char *msg, ...)
{

	char buf[256];
	va_list argptr;

	if (level > log_level)
		return;

	sprintf(buf, "\t[ %s: %5s ] %s: ", modulename, level_str[level], func_name);

	va_start(argptr, msg);
	vsprintf(buf + strlen(buf), msg, argptr);

	printk(buf);
	va_end(argptr);
}


EXPORT_SYMBOL(_log_msg);

