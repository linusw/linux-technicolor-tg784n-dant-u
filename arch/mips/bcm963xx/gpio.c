#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/module.h>

#if defined(CONFIG_BCM963268)
#include <63268_map_part.h>
#elif defined(CONFIG_BCM96318)
#include <6318_map_part.h>
#elif defined(CONFIG_BCM96368)
#include <6368_map_part.h>
#elif defined(CONFIG_BCM96362)
#include <6362_map_part.h>
#elif defined(CONFIG_BCM96328)
#include <6328_map_part.h>
#elif defined(CONFIG_BCM963381)
#include <63381_map_part.h>
#else
#error Chipset not yet supported by the GPIO driver
#endif

#if !defined(CONFIG_BCM963381)

/*
** Gpio Controller
*/
typedef struct BcmGpioControl {
	uint32_t      GPIODir_high;
	uint32_t      GPIODir;
	uint32_t      GPIOio_high;
	uint32_t      GPIOio;
} BcmGpioControl;

#ifdef GPIO
#undef GPIO 
#endif

/* map our own struct on top of GPIO registers to avoid using 64bit regs */
#define GPIO ((volatile BcmGpioControl * const) GPIO_BASE)

#endif

#define GPIO_DIR_OUT 0x0
#define GPIO_DIR_IN  0x1


static DEFINE_SPINLOCK(bcm63xx_gpio_lock);

static int bcm963xx_set_direction(struct gpio_chip *chip, unsigned gpio, int dir)
{
	unsigned long flags;
	uint32_t mask = (1 << gpio % 32);

#if defined(CONFIG_BCM963381)
        volatile uint32 *reg;
        if (gpio / 32 > sizeof(GPIO->GPIODir) / sizeof(GPIO->GPIODir[0])) {
                printk("GPIO access to %d out of range\n", gpio);
                return -1;
        }
        reg = &GPIO->GPIODir[gpio / 32];
#else
        volatile uint32_t *reg;
	if (gpio >= 32)
		reg = &GPIO->GPIODir_high;
	else
		reg = &GPIO->GPIODir;
#endif
        spin_lock_irqsave(&bcm63xx_gpio_lock, flags);
        if (dir == GPIO_DIR_IN)
                *reg &= ~mask;
        else
                *reg |= mask;
        spin_unlock_irqrestore(&bcm63xx_gpio_lock, flags);
	return 0;
}

static int bcm963xx_get(struct gpio_chip *chip, unsigned gpio)
{
#if defined(CONFIG_BCM963381)
        volatile uint32 *reg;
        if (gpio / 32 > sizeof(GPIO->GPIODir) / sizeof(GPIO->GPIODir[0])) {
                printk("GPIO access to %d out of range\n", gpio);
                return -1;
        }
        reg = &GPIO->GPIOio[gpio / 32];
#else
        volatile uint32_t *reg;
	if (gpio >= 32)
		reg = &GPIO->GPIOio_high;
	else
		reg = &GPIO->GPIOio;
#endif
	gpio = 1 << (gpio % 32);

	return (*reg & gpio) != 0;
}

static void bcm963xx_set(struct gpio_chip *chip, unsigned gpio, int value)
{
	unsigned long flags;

#if defined(CONFIG_BCM963381)
        volatile uint32 *reg;
        if (gpio / 32 > sizeof(GPIO->GPIODir) / sizeof(GPIO->GPIODir[0])) {
                printk("GPIO access to %d out of range\n", gpio);
                return;
        }
        reg = &GPIO->GPIOio[gpio / 32];
#else
        volatile uint32_t *reg;
	if (gpio >= 32)
		reg = &GPIO->GPIOio_high;
	else
		reg = &GPIO->GPIOio;
#endif
	gpio = 1 << (gpio % 32);

	spin_lock_irqsave(&bcm63xx_gpio_lock, flags);
	if (value)
		*reg |= gpio;
	else
		*reg &= ~gpio;
	spin_unlock_irqrestore(&bcm63xx_gpio_lock, flags);
}

static int bcm963xx_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return bcm963xx_set_direction(chip, offset, GPIO_DIR_IN);
}

static int bcm963xx_direction_out(struct gpio_chip *chip, unsigned gpio, int value)
{
	bcm963xx_set(chip, gpio, value);
	return bcm963xx_set_direction(chip, gpio, GPIO_DIR_OUT);
}

static struct gpio_chip bcm963xx_gpiochip = {
	.label			= "bcm963xx-gpio",
	.owner			= THIS_MODULE,

	.get                    = bcm963xx_get,
	.set                    = bcm963xx_set,
	.direction_input        = bcm963xx_direction_in,
	.direction_output       = bcm963xx_direction_out,
	.base                   = 0,
	.ngpio                  = GPIO_NUM_MAX,
	.can_sleep              = 0
};

int gpio_to_irq(unsigned gpio)
{
	return __gpio_to_irq(gpio);
}
EXPORT_SYMBOL_GPL(gpio_to_irq);

static int __init bcm963xx_gpio_init(void)
{
	int ret=1;

	ret = gpiochip_add(&bcm963xx_gpiochip);
	if (ret < 0) {
		printk("Failed to register BCM963268 GPIO chip\n");
	}
	return ret;
}

subsys_initcall(bcm963xx_gpio_init);

