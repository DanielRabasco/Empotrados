#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/errno.h>

extern int spkr_init(void);
extern void spkr_exit(void);
extern void spkr_set_frequency(unsigned int frequency);
extern void spkr_on(void);
extern void spkr_off(void);


/*static int minor = 0;
static int n_aperturas_writemode = 0;
static dev_t dev;
static struct cdev cdev;
static struct class *clase;
module_param(minor, int, S_IRUGO);

struct info_mydev {
	struct cdev mydev_cdev;
};

static int spkr_open(struct inode *inode, struct file *filp) {
	struct info_mydev *info_dev = container_of(inode->i_cdev, struct info_mydev, mydev_cdev);
	filp->private_data = info_dev;

	//Check that there is only one call to open in write-mode
	if ((filp->f_mode & FMODE_WRITE) != 0){
		if (n_aperturas_writemode == 0){
			n_aperturas_writemode++;
			printk("WRITE");
		}else{
			return -EBUSY;
		}
	}
	printk(KERN_ALERT "Funcion de apertura");
	return 0;
}


static int spkr_release(struct inode *inode, struct file *filp) {
	//If the open call was in write mode, decrease the counter and allow other write-mode open call
	if ((filp->f_mode & FMODE_WRITE) != 0){
		n_aperturas_writemode--;
	}
	printk(KERN_ALERT "Funcion de liberacion");
	return 0;
}

static ssize_t spkr_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	printk(KERN_ALERT "Funcion de escritura");
	return count;
}

static struct file_operations fops = {
  .owner = THIS_MODULE,
  .open = spkr_open,
  .release = spkr_release,
  .write = spkr_write
};

int spkr_init(void){
	alloc_chrdev_region(&dev, minor, 1, "spkr");
	cdev_init(&cdev, &fops);
	cdev_add(&cdev, dev, 1);
	clase = class_create(THIS_MODULE, "speaker");
	device_create(clase, NULL, dev, NULL, "intspkr");
	printk(KERN_ALERT "MAJOR:%i\n",MAJOR(dev));
	printk(KERN_ALERT "MINOR:%i\n",MINOR(dev));
	return 0;
}

void spkr_exit(void){
	unregister_chrdev_region(dev, 1);
	cdev_del(&cdev);
	device_destroy(clase, dev);
	class_destroy(clase);
}

module_init(spkr_init);
module_exit(spkr_exit);*/