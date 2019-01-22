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
static long ejemplo_ioctl(struct file *filp, unsigned int cmd,
                      unsigned long arg);

// variables
static int minor = 0;
static int n_aperturas_writemode = 0;
static dev_t dev;
static struct cdev cdev;
static struct class *clase;
static struct kfifo fifo;
static int buffer_length = PAGE_SIZE;
static int buffer_threshold = PAGE_SIZE;
static DEFINE_MUTEX(mutex);
spinlock_t lock;
bool dreaming;
int mute;

// parameters
module_param(minor, int, S_IRUGO);
module_param(buffer_length, int, S_IRUGO);
module_param(buffer_threshold, int, S_IRUGO);

//structs
struct info_mydev {
    struct cdev mydev_cdev;
};

struct timer_list timer;

struct info_dispo {
  wait_queue_head_t lista_bloq;
  // ...............
} info;

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
static int ejemplo_fsync(struct file *filp, loff_t start, loff_t end, int datasync);
#else
static int ejemplo_fsync(struct file *filp, int datasync);
#endif

static int spkr_open(struct inode *inode, struct file *filp) {
        
	struct info_mydev *info_dev = container_of(inode->i_cdev, struct info_mydev, mydev_cdev);
	filp->private_data = info_dev;

	//Check that there is only one call to open in write-mode
	if ((filp->f_mode & FMODE_WRITE) != 0){
		if (n_aperturas_writemode == 0){
			n_aperturas_writemode++;
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

    
    printk(KERN_ALERT "Funcion de ESCRITURA");

    unsigned int count_copy = 0;
    unsigned int copied;

    // Bloqueo del proceso
    printk(KERN_ALERT "Blockingmutex");
    if (mutex_lock_interruptible(&mutex)){
        return -ERESTARTSYS;
    }

    while (count_copy < count){

        timer.data = count - count_copy;

        printk(KERN_ALERT "Blocked list");;
        if(wait_event_interruptible(info.lista_bloq, kfifo_avail(&fifo) > 0) != 0){
            return -ERESTARTSYS;
        }
        printk(KERN_ALERT "Unblocked list");

        if (kfifo_from_user(&fifo, buf+count_copy, count, &copied) != -EFAULT) {
            count_copy += copied;

            if(!dreaming && (kfifo_len(&fifo) > 3)){
                printk(KERN_ALERT "Playing by writting operation");
                spkr_play();
            }
            printk(KERN_ALERT "Unblockingmutex");
            mutex_unlock(&mutex);
        }
        else{
            return -EFAULT;
        }
    }
    return count;
}

static struct file_operations fops = {
  .owner = THIS_MODULE,
  .open = spkr_open,
  .release = spkr_release,
  .write = spkr_write,
  .fsync = ejemplo_fsync,
  .unlocked_ioctl =  ejemplo_ioctl
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
    mute = 0;

    // Rounding up proccess
    if (buffer_length < 1) {
        return 0;
     }else{
        buffer_length--; 
      
        int cont;
        cont = 0;
        while (buffer_length > 1){
            buffer_length = (int) buffer_length/2;
            cont++;
        } 
      
        int i;
        i = 0;
        buffer_length = 1;
        while (i < cont + 1){
            buffer_length = buffer_length*2;
            i++;
        }
    }
    
    if(kfifo_alloc(&fifo, buffer_length, GFP_USER)){
        return -ENOMEM;
    }
    
    if(buffer_threshold > buffer_length) {
	buffer_threshold = buffer_length;
    }
    
    spin_lock_init(&lock);
        
    spkr_init();
    /*spkr_on();
    spkr_set_frequency(1047);*/
    return 0;
}

void speaker_exit(void){
	unregister_chrdev_region(dev, 1);
	cdev_del(&cdev);
	device_destroy(clase, dev);
	class_destroy(clase);
	kfifo_free(&fifo);
	del_timer_sync(&timer);
	spkr_off();
	spkr_exit();
	printk(KERN_ALERT "Exiting");
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,0,0)
static int ejemplo_fsync(struct file *filp, loff_t start, loff_t end, int datasync) {
    wait_event_interruptible(info.lista_bloq, kfifo_is_empty(&fifo));
    return 0;
}
#else
static int ejemplo_fsync(struct file *filp, int datasync){
    wait_event_interruptible(info.lista_bloq, kfifo_is_empty(&fifo));
    return 0;
}
#endif


static void spkr_play(void) {
    char sound[4];
    int data;
    int frequency;
    int time;
    dreaming = true;
    
    // The ioctl operation could be reseting the fifo
    spin_lock_bh(&lock);
    data = kfifo_out(&fifo, sound, 4);
    spin_unlock_bh(&lock);
    if(data != 0){
        // los dos primeros bytes especifican la frecuencia
        frequency = (int) ((sound[1] << 8) + sound[0]);
        printk(KERN_ALERT "Frecuency: %d", frequency);
        
        // los dos bytes siguientes especifican la duracion
        time = (int) ((sound[3] << 8) + sound[2]);
        printk(KERN_ALERT "Time: %d", time);

        if(frequency == 0){
            spkr_off();
        } else {
            spkr_set_frequency(frequency);
            
            // Solo reproducimos si no esta muteado
            if(mute == 0){
                spkr_on();
            }
        }
        
        // establecemos el tiempo del temporizador para cuando haya acabado el sonido
        timer.expires = jiffies + msecs_to_jiffies(time);
        add_timer(&timer);
    }
}

static void temp_treatment(unsigned long d) {
    // If we have sounds to play
    if(kfifo_len(&fifo) > 3){
        printk(KERN_ALERT "Playing by temp_treatment");
        spkr_play();
    }
    // If not disable the spkr
    else{
        spkr_off();
        dreaming = false;
    }

    int min;
    min = 0;
    if (d > buffer_threshold){
	min = buffer_threshold;
        }else{
            min = d;
        }
    
    // desbloqueará a un proceso escritor si el hueco en la cola o bien es suficiente para
    // que el proceso pueda completar su petición, o bien es mayor o igual que el umbral recibido como parámetro
    if(kfifo_is_empty(&fifo) || kfifo_avail(&fifo) >= min) {
        printk(KERN_ALERT "Temp UNBLOCKING");
        wake_up_interruptible(&info.lista_bloq);
    }
}

static long ejemplo_ioctl(struct file *filp, unsigned int cmd,
                      unsigned long arg)
{
    int *value;
    int local;
    int SPKR_SET_MUTE_STATE;
    int SPKR_GET_MUTE_STATE;
    int SPKR_RESET;
    SPKR_SET_MUTE_STATE = _IOW('9',1,int*);
    SPKR_GET_MUTE_STATE = _IOR('9',2,int*);
    SPKR_RESET = _IO('9',3);
            
    if(cmd == SPKR_GET_MUTE_STATE){
        put_user(mute, (int*)arg);
        }
        else if (cmd == SPKR_SET_MUTE_STATE){
                value = (int*)arg;
                get_user(local, value/*&(int*)arg*/);
                if(local == 0){
                    mute = 1;
                    spkr_off();
                }else{
                    mute= 0;
                    spkr_on();
                }
            } 
            else if(cmd == SPKR_RESET){
                // el vaciado no se produce hasta que se completa el procesado del sonido en curso
                spin_lock_bh(&lock);
                kfifo_reset(&fifo);
                spin_unlock_bh(&lock);
            }

  return 0;
}



module_init(speaker_init);
module_exit(speaker_exit);
