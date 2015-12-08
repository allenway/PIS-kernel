#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>

struct wake_lock wakelock;

static int __init wakelock_init(void)
{
	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND,
		       "hold_the_android_wakelock");
	wake_lock(&wakelock);
	
}

static void __exit wakelock_exit(void)
{
	wake_unlock(&wakelock);	
}

module_init(wakelock_init);
module_exit(wakelock_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("calvin");
MODULE_DESCRIPTION("hold_the_android_wakelock");
MODULE_ALIAS("platform:hold_the_android_wakelock");
