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
#include <linux/timer.h>
MODULE_LICENSE("Dual BSD/GPL");

// io methods
extern int spkr_init(void);
extern void spkr_exit(void);
extern void spkr_set_frequency(unsigned int frequency);
extern void spkr_on(void);
extern void spkr_off(void);

// functions
static void spkr_play(void);
static void temp_treatment(unsigned long d);
unsigned int minimum(unsigned int threshold, unsigned int bytes_left);
unsigned long upper_power_of_two(unsigned long v);

// variables
static int minor = 0;
static int n_aperturas_writemode = 0;
static dev_t dev;
static struct cdev cdev;
static struct class *clase;
static struct kfifo fifo;
static int buffer_length = PAGE_SIZE;
static DEFINE_MUTEX(mutex);
spinlock_t lock;
bool dreaming;
volatile int mute_sound;

// parameters
module_param(minor, int, S_IRUGO);
module_param(buffer_length, int, S_IRUGO);

//structs
struct info_mydev {
	struct cdev mydev_cdev;
};

struct timer_list timer;

struct info_dispo {
  wait_queue_head_t lista_bloq;
  // ...............
} info;

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

    unsigned int count_copy = 0;
    unsigned int copied;

    // Bloqueo del proceso
    if (mutex_lock_interruptible(&mutex)){
        return -ERESTARTSYS;
    }

    while (count_copy < count){

        timer.data = count - count_copy;

        printk(KERN_ALERT "ANTES DEL BLOQUEO");
        if(wait_event_interruptible(info.lista_bloq, kfifo_avail(&fifo) > 0) != 0){
            return -ERESTARTSYS;
        }

        if (kfifo_from_user(&fifo, buf+count_copy, count, &copied) != -EFAULT) {
            count_copy += copied;

            if(!dreaming && (kfifo_len(&fifo) >= 4)){
                spkr_play();
            }

            mutex_unlock(&mutex);
            return count_copy;
        }
        else{
            return -EFAULT;
        }
    }

	printk(KERN_ALERT "Funcion de escritura");
	return count;
}

static struct file_operations fops = {
  .owner = THIS_MODULE,
  .open = spkr_open,
  .release = spkr_release,
  .write = spkr_write
};

int speaker_init(void){
	alloc_chrdev_region(&dev, minor, 1, "spkr");
	cdev_init(&cdev, &fops);
	cdev_add(&cdev, dev, 1);
	clase = class_create(THIS_MODULE, "speaker");
	device_create(clase, NULL, dev, NULL, "intspkr");
	printk(KERN_ALERT "MAJOR:%i\n",MAJOR(dev));
	printk(KERN_ALERT "MINOR:%i\n",MINOR(dev));

    // Step 4 of the project initialization
	init_timer(&timer);
	timer.function = temp_treatment;
    timer.data = 0;
	init_waitqueue_head(&info.lista_bloq);
	mute_sound = 0;

	buffer_length = upper_power_of_two(buffer_length);
    if(kfifo_alloc(&fifo, buffer_length, GFP_USER)){
        return -ENOMEM;
    }
	return 0;
}

void speaker_exit(void){
	unregister_chrdev_region(dev, 1);
	cdev_del(&cdev);
	device_destroy(clase, dev);
	class_destroy(clase);
	kfifo_free(&fifo);
}

unsigned long upper_power_of_two(unsigned long v) {
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v++;
  return v;
}

static void spkr_play(void) {
    unsigned char sound_data[4];
    unsigned int frequency;
    unsigned int delay;
    unsigned int elements_copied;

    dreaming = true;
    spin_lock_bh(&lock);
    elements_copied = kfifo_out(&fifo, sound_data, 4);
    if(elements_copied == 0){
        spin_unlock_bh(&lock);
        return;
    }
    spin_unlock_bh(&lock);

    frequency = (unsigned int) ((sound_data[1] << 8) + sound_data[0]);
    delay = (unsigned int) ((sound_data[3] << 8) + sound_data[2]);
    printk(KERN_ALERT "Frecuencia obtenida: %d", frequency);
    printk(KERN_ALERT "Delay obtenido: %d", delay);

    if(frequency == 0){
        spkr_off();
    } else {
        spkr_set_frequency(frequency);
        if(mute_sound == 0){
            spkr_on();
        }
      }
    timer.expires = jiffies + msecs_to_jiffies(delay);
    add_timer(&timer);
}

static void temp_treatment(unsigned long d) {
    // If we have sounds to play
    if(kfifo_len(&fifo) >= 4){
        spkr_play();
    }
    // If not disable the spkr
    else{
        spkr_off();
        dreaming = false;
    }

    // desbloqueará a un proceso escritor si el hueco en la cola o bien es suficiente para
    // que el proceso pueda completar su petición, o bien es mayor o igual que el umbral recibido como parámetro
    if(kfifo_is_empty(&fifo) || kfifo_avail(&fifo) >= minimum(buffer_threshold, bytes_left)){
        wake_up_interruptible(&info.lista_bloq);
    }
}

unsigned int minimum(unsigned int threshold, unsigned int bytes_left)
{
  if(threshold < bytes_left)
    return threshold;
  return bytes_left;
}



module_init(speaker_init);
module_exit(speaker_exit);
