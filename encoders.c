#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/cdev.h> 
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/vmalloc.h>
#include <mach/platform.h>
#include <asm/uaccess.h> 
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/i2c.h>
 
#define DRIVER_AUTHOR "Dan Hastings"
#define DRIVER_DESC   "Robot Sensors"

#define GPIO_LEN 0xb4

#define FIFO 0x74
#define COUNT_ADDR 0x72
#define USER_CNTL 0x6A
#define INT_STATUS 0x3A

#define GPIO_LEV (0x34/4)
 
// we want GPIO_17 (pin 11 on P5 pinout raspberry pi rev. 2 board)
// to generate interrupt
#define ENCODER1A                17
#define ENCODER1B                18
#define LEFTMASK ((1<<ENCODER1A) | (1<<ENCODER1B))
#define ENCODER2A                27
#define ENCODER2B                22
#define RIGHTMASK ((1<<ENCODER2A) | (1<<ENCODER2B))
#define INT_PIN 23
 
 
// below is optional, used in more complex code, in our case, this could be
// NULL
#define GPIO_ANY_GPIO_DEVICE_DESC    "some_device"
 
 
/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/
short int got_gpio[5];

static struct i2c_client *my_client;
 
static volatile uint32_t *gpio_reg;

static dev_t devno;
static struct class *cl;
static struct cdev my_cdev;
static int my_major;

static volatile int left, right;

DEFINE_MUTEX(lock);

int data_ready = 0;

DECLARE_WAIT_QUEUE_HEAD(wait_q);
 
static int dev_open(struct inode *inod, struct file *fil);
static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);

static int __devinit mpu_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	my_client = client;

	printk("I think its ok\n");

	return 0;
}

static struct i2c_device_id pi_mpu_id[] = {
	{"pi_mpu", 0},
	{}
};

static struct i2c_driver pi_mpu_driver = {
	.driver = {
							.name = "pi-mpu",
						},
	.probe = mpu_probe,
	.id_table = pi_mpu_id,
};

static struct file_operations fops =
{
	.open = &dev_open,
	.read = &dev_read,
	.write = NULL,
	.release = NULL,
	.unlocked_ioctl = NULL,
	.compat_ioctl = NULL,
};

void mpu_disabledmp(void)
{
  uint8_t message[2];
  int ret;

	uint8_t *ctl_addr = message;
	uint8_t *ctl_reg = message + 1;
  *ctl_reg = 0;
  *ctl_addr = USER_CNTL;

	struct i2c_msg ctl_read[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 1,
			.buf = ctl_addr,
		},
		{
			.addr = my_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = ctl_reg,
		},
	};

	struct i2c_msg ctl_write[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 2,
			.buf = message,
		},
	};

	ret = i2c_transfer(my_client->adapter, ctl_read, 2);

//  printk("%02X, %d\n", ctl_reg, ret);

	*ctl_reg &= !(1<<7);

	ret = i2c_transfer(my_client->adapter, ctl_write, 1);

  printk("%02X, %d\n", *ctl_reg, ret);
}

void mpu_enabledmp(void)
{
  uint8_t message[2];
  int ret;

	uint8_t *ctl_addr = message;
	uint8_t *ctl_reg = message + 1;
  *ctl_reg = 0;
  *ctl_addr = USER_CNTL;

	struct i2c_msg ctl_read[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 1,
			.buf = ctl_addr,
		},
		{
			.addr = my_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = ctl_reg,
		},
	};

	struct i2c_msg ctl_write[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 2,
			.buf = message,
		},
	};

	ret = i2c_transfer(my_client->adapter, ctl_read, 2);

//  printk("%02X, %d\n", ctl_reg, ret);

	*ctl_reg |= (1<<7);

	ret = i2c_transfer(my_client->adapter, ctl_write, 1);

  printk("%02X, %d\n", *ctl_reg, ret);
}

void mpu_resetfifo(void)
{
  uint8_t message[2];
  int ret;

	uint8_t *ctl_addr = message;
	uint8_t *ctl_reg = message + 1;
  *ctl_reg = 0;
  *ctl_addr = USER_CNTL;

	struct i2c_msg ctl_read[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 1,
			.buf = ctl_addr,
		},
		{
			.addr = my_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = ctl_reg,
		},
	};

	struct i2c_msg ctl_write[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 2,
			.buf = message,
		},
	};

	ret = i2c_transfer(my_client->adapter, ctl_read, 2);

//  printk("%02X, %d\n", ctl_reg, ret);

	*ctl_reg |= (1<<2);

	ret = i2c_transfer(my_client->adapter, ctl_write, 1);

  printk("%02X, %d\n", *ctl_reg, ret);
}

uint8_t mpu_getintstatus(void)
{
	uint8_t int_status_addr = INT_STATUS;
	uint8_t int_status_reg;

	struct i2c_msg int_status_read[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &int_status_addr,
		},
		{
			.addr = my_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &int_status_reg,
		},
	};

	i2c_transfer(my_client->adapter, int_status_read, 2);

	return int_status_reg;
}

int mpu_getfifocount(void)
{
	uint8_t count_addr = COUNT_ADDR;
	uint16_t count;
	uint8_t count_buffer[2];

	struct i2c_msg fifo_read[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &count_addr,
		},
		{
			.addr = my_client->addr,
			.flags = I2C_M_RD,
			.len = 2,
			.buf = count_buffer,
		},
	};

	i2c_transfer(my_client->adapter, fifo_read, 2);

	count = (((uint16_t)count_buffer[0]) << 8) | count_buffer[1];

	return count;
}

static uint8_t input_buffer[42];
static uint8_t read_buffer[42];

void mpu_readfifo(void)
{
	uint8_t fifo = FIFO;

	int fifo_count = mpu_getfifocount();

	struct i2c_msg fifo_read[] = {
		{
			.addr = my_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &fifo,
		},
		{
			.addr = my_client->addr,
			.flags = I2C_M_RD,
			.len = 42,
			.buf = input_buffer,
		},
	};


  if (fifo_count == 1024)
  {
    mpu_resetfifo();
  }
  else 
  {
    while (fifo_count >= 42)
    {
  		i2c_transfer(my_client->adapter, fifo_read, 2);

  		mutex_lock(&lock);

  		memcpy(&read_buffer, &input_buffer, sizeof(input_buffer));

  		mutex_unlock(&lock);

  		fifo_count -= 42;

      fifo_count = mpu_getfifocount();
    }
	}

 	data_ready = 1;
	wake_up(&wait_q);
}

void mpu_tasklet(unsigned long dummy)
{
	mpu_readfifo();
}

static struct workqueue_struct *queue;
static struct work_struct mpu_wq;

/****************************************************************************/
/* IRQ handler - fired on interrupt                                         */
/****************************************************************************/
static irqreturn_t mpu_handler(int irq, void *dev_id) {
	 queue_work(queue, &mpu_wq);

   return IRQ_HANDLED;
}

static irqreturn_t enc1a_handler(int irq, void *dev_id) {
 
   unsigned long flags;
	 uint32_t pins;   

   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);

	 pins = gpio_reg[GPIO_LEV] & LEFTMASK;

	 if (pins & (1<<ENCODER1A))
	 {
		 if (pins & (1<<ENCODER1B))
		 {
			 left--;
		 }
		 else
		 {
			 left++;
		 }
	 }
	 else
	 {
		 if (pins & (1<<ENCODER1B))
		 {
			 left++;
		 }
		 else
		 {
			 left--;
		 }
	 }
 
   // restore hard interrupts
   local_irq_restore(flags);

   return IRQ_HANDLED;
}

static irqreturn_t enc1b_handler(int irq, void *dev_id) {
 
   unsigned long flags;
	 uint32_t pins;   

   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);

	 pins = gpio_reg[GPIO_LEV] & LEFTMASK;

	 if (pins & (1<<ENCODER1B))
	 {
		 if (pins & (1<<ENCODER1A))
		 {
			 left++;
		 }
		 else
		 {
			 left--;
		 }
	 }
	 else
	 {
		 if (pins & (1<<ENCODER1A))
		 {
			 left--;
		 }
		 else
		 {
			 left++;
		 }
	 }
 
   // restore hard interrupts
   local_irq_restore(flags);
 
   return IRQ_HANDLED;
}

static irqreturn_t enc2a_handler(int irq, void *dev_id) {
 
   unsigned long flags;
	 uint32_t pins;   

   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);

	 pins = gpio_reg[GPIO_LEV] & RIGHTMASK;

	 if (pins & (1<<ENCODER2A))
	 {
		 if (pins & (1<<ENCODER2B))
		 {
			 right++;
		 }
		 else
		 {
			 right--;
		 }
	 }
	 else
	 {
		 if (pins & (1<<ENCODER2B))
		 {
			 right--;
		 }
		 else
		 {
			 right++;
		 }
	 }
 

   // restore hard interrupts
   local_irq_restore(flags);
 
   return IRQ_HANDLED;
}

static irqreturn_t enc2b_handler(int irq, void *dev_id) {
 
   unsigned long flags;
	 uint32_t pins;   

   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);

	 pins = gpio_reg[GPIO_LEV] & RIGHTMASK;

	 if (pins & (1<<ENCODER2B))
	 {
		 if (pins & (1<<ENCODER2A))
		 {
			 right--;
		 }
		 else
		 {
			 right++;
		 }
	 }
	 else
	 {
		 if (pins & (1<<ENCODER2A))
		 {
			 right++;
		 }
		 else
		 {
			 right--;
		 }
	 }
 
   // restore hard interrupts
   local_irq_restore(flags);
 
   return IRQ_HANDLED;
}
/****************************************************************************/
/* This function configures interrupts.                                     */
/****************************************************************************/
void r_int_config(void) {

   unsigned long flags;

	 local_irq_save(flags);

	 if (request_irq(gpio_to_irq(ENCODER1A), enc1a_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_PROBE_SHARED, "enc1a", NULL))
	{
		printk("trouble getting interrupt\n");
	}
	else
	{
		printk("got it\n");
		got_gpio[0] = 1;
	}

  if (request_irq(gpio_to_irq(ENCODER1B), enc1b_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_PROBE_SHARED, "enc1b", NULL))
	{
		printk("trouble getting interrupt\n");
	}
	else
	{
		printk("got it\n");
		got_gpio[1] = 1;
	}

  if (request_irq(gpio_to_irq(ENCODER2A), enc2a_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_PROBE_SHARED, "enc2a", NULL))
	{
		printk("trouble getting interrupt\n");
	}
	else
	{
		printk("got it\n");
		got_gpio[2] = 1;
	}

  if (request_irq(gpio_to_irq(ENCODER2B), enc2b_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_PROBE_SHARED, "enc2b", NULL))
	{
		printk("trouble getting interrupt\n");
	}
	else
	{
		printk("got it\n");
		got_gpio[3] = 1;
	}

  if (request_irq(gpio_to_irq(INT_PIN), mpu_handler, IRQF_TRIGGER_RISING | IRQF_PROBE_SHARED, "enc2b", NULL))
	{
		printk("trouble getting interrupt\n");
	}
	else
	{
		printk("got it\n");
		got_gpio[4] = 1;
	}

	local_irq_restore(flags);

}
 
 
/****************************************************************************/
/* This function releases interrupts.                                       */
/****************************************************************************/
void r_int_release(void) {

	 if (got_gpio[0])
		 free_irq(gpio_to_irq(ENCODER1A), NULL);

	 if (got_gpio[1])
		 free_irq(gpio_to_irq(ENCODER1B), NULL);

	 if (got_gpio[2])
		 free_irq(gpio_to_irq(ENCODER2A), NULL);

	 if (got_gpio[3])
		 free_irq(gpio_to_irq(ENCODER2B), NULL);

 	 if (got_gpio[4])
		 free_irq(gpio_to_irq(INT_PIN), NULL);

 return;
}
 
static int dev_open(struct inode *inod, struct file *fil)
{

	return 0;
}

static ssize_t dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret = 0;
	int dummy;

	memset(buf, 0, count);

	wait_event_interruptible(wait_q, data_ready);
	data_ready = 0;

	if (count >= 50)
	{
		ret = 50;

		mutex_lock(&lock);

		dummy = copy_to_user(buf, read_buffer, 42);

		mutex_unlock(&lock);

		dummy = copy_to_user(buf + 42, &left, 4);
		dummy = copy_to_user(buf + 46, &right, 4);
	}

	return ret;
}
 
/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/
int r_init(void) {
 
	 int res;
   printk(KERN_NOTICE "Hello !\n");

	 gpio_reg = (uint32_t *) ioremap(GPIO_BASE, GPIO_LEN);

	 queue = create_workqueue("read");

	 INIT_WORK(&mpu_wq, (void (*)(void *)) mpu_tasklet);

	 i2c_add_driver(&pi_mpu_driver);

   r_int_config();
	
   mpu_enabledmp();

	 mpu_resetfifo();

	 res = alloc_chrdev_region(&devno, 0, 1, "encoders");

	 if (res < 0)
	 {
		 return res;
	 }
	 my_major = MAJOR(devno);
	 cdev_init(&my_cdev, &fops);
	 my_cdev.owner = THIS_MODULE;
	 my_cdev.ops = &fops;
	 res = cdev_add(&my_cdev, MKDEV(my_major, 0), 1);
	 if (res)
	 {
		 unregister_chrdev_region(devno, 1);
		 return res;
	 }
	 else
	 {
		 cl =class_create(THIS_MODULE, "encoder class");

		 device_create(cl, NULL, devno, NULL, "encoders");
	 }
 
	 left = 0;
	 right = 0;

   return 0;
}
 
void r_cleanup(void) {
   printk(KERN_NOTICE "Goodbye\n");
   r_int_release();

	 iounmap(gpio_reg);

   mpu_disabledmp();

	 i2c_del_driver(&pi_mpu_driver);
 
	 cdev_del(&my_cdev);
	 unregister_chrdev_region(devno, 1);

	 device_destroy(cl, devno);
	 class_destroy(cl);

   return;
}
 
 
module_init(r_init);
module_exit(r_cleanup);
 
 
/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
