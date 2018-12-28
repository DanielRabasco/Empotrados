#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
MODULE_LICENSE("Dual BSD/GPL");

void spkr_init(void) {
	void __iomem *regs_div = ioremap(0x3F1010A4, 4);
	void __iomem *regs_ctl = ioremap(0x3F1010A0, 4);

	void __iomem *regs_gpio = ioremap(0x3f200100, 4);
	uint32_t val_gpio = ioread32(regs_gpio); 

	// region for the pwm control
	void __iomem *reg_pwm_ctl = ioremap(0x3f20C000, 4);
	uint32_t val_pwm_ctl = ioread32(reg_pwm_ctl);

	// region for the pwm range
	void __iomem *reg_pwm_rng = ioremap(0x3f20C010, 4);
	uint32_t val_pwm_rng = ioread32(reg_pwm_rng);

	// region for the pwm data
	void __iomem *reg_pwm_data = ioremap(0x3f20C014, 4);
	uint32_t val_pwm_data = ioread32(reg_pwm_data);

	// 
	iowrite32(0x5A010000, regs_div);
	iowrite32(0x5A000011, regs_ctl);

	// Reseting the bits (23, 24 and 25) values and writing 010 in them
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
	iounmap(0x3F1010A4);
	iounmap(0x3F1010A0);
	iounmap(0x3f200100);
	iounmap(0x3f20C000);
	iounmap(0x3f20C010);
	iounmap(0x3f20C014);

	printk(KERN_INFO "spkr exit\n");
	
}

void spkr_set_frequency(unsigned int frequency) {
	printk(KERN_INFO "spkr set frequency: %d\n", frequency);
}

void spkr_on(void) {
	
	// region for the pwm control
	void __iomem *reg_pwm_ctl = ioremap(0x3f20C000, 4);
	uint32_t val_pwm_ctl = ioread32(reg_pwm_ctl);

	// Reseting bit 0 and writing 1 in it(in the pwm control region) to activate
	val_pwm_ctl &= ~(0x1 << 0);
	val_pwm_ctl |= (0x1 << 0);
	iowrite32(val_pwm_ctl, reg_pwm_ctl);
	printk(KERN_INFO "spkr ON\n");
}
void spkr_off(void) {
	
	// region for the pwm control
	void __iomem *reg_pwm_ctl = ioremap(0x3f20C000, 4);
	uint32_t val_pwm_ctl = ioread32(reg_pwm_ctl);

	// Reseting bit 0 and writing 0 in it(in the pwm control region) to deactivate
	val_pwm_ctl &= ~(0x0 << 0);
	val_pwm_ctl |= (0x0 << 0);
	iowrite32(val_pwm_ctl, reg_pwm_ctl);
	printk(KERN_INFO "spkr OFF\n");
}
