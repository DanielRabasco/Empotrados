#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/io.h>
MODULE_LICENSE("Dual BSD/GPL");

volatile void __iomem *regs_div;
volatile void __iomem *regs_ctl;
volatile void __iomem *regs_gpio;
volatile void __iomem *reg_pwm_ctl;
volatile void __iomem *reg_pwm_rng;
volatile void __iomem *reg_pwm_data;

uint32_t val_gpio;
uint32_t val_pwm_ctl;
uint32_t val_pwm_rng;
uint32_t val_pwm_data;

void spkr_init(void) {
	
	regs_div = ioremap(0x3F1010A4, 4);
	regs_ctl = ioremap(0x3F1010A0, 4);

	regs_gpio = ioremap(0x3f200004, 4);
	val_gpio = ioread32(regs_gpio); 
	printk(KERN_ALERT "GPIO: %x\n", val_gpio);

	// region for the pwm control
	reg_pwm_ctl = ioremap(0x3f20C000, 4);
	val_pwm_ctl = ioread32(reg_pwm_ctl);
	printk(KERN_ALERT "PWMCTL: %x\n", val_pwm_ctl);

	// region for the pwm range
	reg_pwm_rng = ioremap(0x3f20C010, 4);
	val_pwm_rng = ioread32(reg_pwm_rng);

	// region for the pwm data
	reg_pwm_data = ioremap(0x3f20C014, 4);
	val_pwm_data = ioread32(reg_pwm_data);
	printk(KERN_ALERT "RANGE: %x\n", val_pwm_rng);
	printk(KERN_ALERT "DATA: %x\n", val_pwm_data);
	// 
	
	printk(KERN_ALERT "REGS_CTL BEFORE: %x\n", ioread32(regs_ctl));
	iowrite32(0x5A010000, regs_div);
	iowrite32(0x5A000011, regs_ctl);
	uint32_t a = ioread32(regs_ctl);
	printk(KERN_ALERT "REGS_CTL AFTER: %x\n", a);

	// Reseting the bits (24, 25 and 26) values and writing 010 in them
	val_gpio &= ~(0x7 << 24);
	val_gpio |= (0x2 << 24);
	iowrite32(val_gpio, regs_gpio);

	// Reseting bit 7 and writing 1 in it(in the pwm control region)
	val_pwm_ctl &= ~(0x1 << 7);
	val_pwm_ctl |= (0x1 << 7);
	iowrite32(val_pwm_ctl, reg_pwm_ctl);

	printk(KERN_INFO "spkr init\n");
}

void spkr_exit(void) {
	iounmap(regs_div);
	iounmap(regs_ctl);
	iounmap(regs_gpio);
	iounmap(reg_pwm_ctl);
	iounmap(reg_pwm_rng);
	iounmap(reg_pwm_data);

	printk(KERN_INFO "spkr exit\n");
	
}

void spkr_set_frequency(unsigned int frequency) {
	uint32_t range;
	range = 1200000/frequency;
	printk(KERN_INFO "Value before setting: %x\n", ioread32(reg_pwm_rng));
	iowrite32(range, reg_pwm_rng);
	iowrite32(range/2, reg_pwm_data);
	printk(KERN_INFO "Value Afte setting: %x\n", ioread32(reg_pwm_rng));
	printk(KERN_INFO "spkr set frequency: %d\n", frequency);
}

void spkr_on(void) {
	
	val_pwm_ctl = ioread32(reg_pwm_ctl);
	
	// Reseting bit 0 and writing 1 in it(in the pwm control region) to activate
	printk(KERN_ALERT "REGS_PWM_CTL BEFORE: %x\n", ioread32(regs_ctl));
	val_pwm_ctl &= ~(0x1 << 0);
	val_pwm_ctl |= (0x1 << 0);
	printk(KERN_ALERT "REGS_PWM_CTL AFTER ON: %x\n", ioread32(regs_ctl));
	iowrite32(val_pwm_ctl, reg_pwm_ctl);
	printk(KERN_INFO "spkr ON\n");
}
void spkr_off(void) {
	
	val_pwm_ctl = ioread32(reg_pwm_ctl);

	// Reseting bit 0 and writing 0 in it(in the pwm control region) to deactivate
	printk(KERN_ALERT "REGS_PWM_CTL BEFORE: %x\n", ioread32(regs_ctl));
	val_pwm_ctl &= ~(0x1 << 0);
	val_pwm_ctl |= (0x0 << 0);
	iowrite32(val_pwm_ctl, reg_pwm_ctl);
	printk(KERN_ALERT "REGS_PWM_CTL AFTER OFF: %x\n", ioread32(regs_ctl));
	printk(KERN_INFO "spkr OFF\n");
}

