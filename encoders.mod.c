#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xda9e78e9, "module_layout" },
	{ 0x85305f03, "class_destroy" },
	{ 0x4b8b420c, "device_destroy" },
	{ 0x74738f6d, "cdev_del" },
	{ 0x30fbf353, "i2c_del_driver" },
	{ 0xc2165d85, "__arm_iounmap" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0x148887b7, "device_create" },
	{ 0xb4c75863, "__class_create" },
	{ 0xe3321d40, "cdev_add" },
	{ 0x60c2503e, "cdev_init" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0x20082fb7, "i2c_register_driver" },
	{ 0x43a53735, "__alloc_workqueue_key" },
	{ 0x40a6f522, "__arm_ioremap" },
	{ 0xf20dabd8, "free_irq" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0xb9e52429, "__wake_up" },
	{ 0xb5e48452, "i2c_transfer" },
	{ 0x33543801, "queue_work" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0xfa2a45e, "__memzero" },
	{ 0x67c2fa54, "__copy_to_user" },
	{ 0x62b72b0d, "mutex_unlock" },
	{ 0xe16b893b, "mutex_lock" },
	{ 0x8893fa5d, "finish_wait" },
	{ 0x75a17bed, "prepare_to_wait" },
	{ 0x1000e51, "schedule" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0x27e1a049, "printk" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "56356CA5DA486FDDC3E6A01");
