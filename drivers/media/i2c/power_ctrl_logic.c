// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020-2021 Intel Corporation.

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include "power_ctrl_logic.h"
#include "gpiolib.h"

static struct power_ctrl_logic pcl = {
	.reset_gpio = NULL,
	.powerdn_gpio = NULL,
	.clocken_gpio = NULL,
	.indled_gpio = NULL,
	.power_on = false,
	.gpio_ready = false,
};

#define PATH_GPI0 "\\_SB.GPI0"
#define PATH_DSC0 "\\_SB.PC00.DSC0"

static const guid_t int3472_clk_guid =
	GUID_INIT(0x82c0d13a, 0x78c5, 0x4244,
		  0x9b, 0xb1, 0xeb, 0x8b, 0x53, 0x9a, 0x8d, 0x11);

/**
 * gpiochip_get_desc - get the GPIO descriptor corresponding to the given
 *                     hardware number for this chip
 * @gc: GPIO chip
 * @hwnum: hardware number of the GPIO for this chip
 *
 * Returns:
 * A pointer to the GPIO descriptor or ``ERR_PTR(-EINVAL)`` if no GPIO exists
 * in the given chip for the specified hardware number.
 */
struct gpio_desc *gpiochip_get_desc(struct gpio_chip *gc,
                                    unsigned int hwnum)
{
        struct gpio_device *gdev = gc->gpiodev;

        if (hwnum >= gdev->ngpio)
                return ERR_PTR(-EINVAL);

        return &gdev->descs[hwnum];
}

static int acpi_gpiochip_find(struct gpio_chip *gc, void *data)
{
        if (!gc->parent)
                return false;

        return ACPI_HANDLE(gc->parent) == data;
}

/**
 * _acpi_get_gpiod() - Translate ACPI GPIO pin to GPIO descriptor usable with GPIO API
 * @path:       ACPI GPIO controller full path name, (e.g. "\\_SB.GPO1")
 * @pin:        ACPI GPIO pin number (0-based, controller-relative)
 *
 * Return: GPIO descriptor to use with Linux generic GPIO API, or ERR_PTR
 * error value. Specifically returns %-EPROBE_DEFER if the referenced GPIO
 * controller does not have GPIO chip registered at the moment. This is to
 * support probe deferral.
 */
static struct gpio_desc *_acpi_get_gpiod(char *path, int pin)
{
        struct gpio_chip *chip;
        acpi_handle handle;
        acpi_status status;

        status = acpi_get_handle(NULL, path, &handle);
        if (ACPI_FAILURE(status))
                return ERR_PTR(-ENODEV);

        chip = gpiochip_find(handle, acpi_gpiochip_find);
        if (!chip)
                return ERR_PTR(-EPROBE_DEFER);

        return gpiochip_get_desc(chip, pin);
}


static int power_ctrl_logic_probe(struct pci_dev *pdev,
				  const struct pci_device_id *id)
{
	acpi_status status;

	status = acpi_get_handle(NULL, PATH_DSC0, &pcl.clk_handle);
	if (ACPI_FAILURE(status))
		return -EINVAL;

	if (!acpi_check_dsm(pcl.clk_handle, &int3472_clk_guid, 0x00, 0x1))
		return -33;

	pcl.reset_gpio = _acpi_get_gpiod(PATH_GPI0, 22);
	if (IS_ERR(pcl.reset_gpio))
          printk("ERR|%s|%d| GPIO", __FUNCTION__, __LINE__);

	pcl.powerdn_gpio = _acpi_get_gpiod(PATH_GPI0, 20);
	if (IS_ERR(pcl.powerdn_gpio))
          printk("ERR|%s|%d| GPIO", __FUNCTION__, __LINE__);

	pcl.indled_gpio = _acpi_get_gpiod(PATH_GPI0, 19);
	if (IS_ERR(pcl.indled_gpio))
          printk("ERR|%s|%d| GPIO", __FUNCTION__, __LINE__);

	mutex_lock(&pcl.status_lock);
	pcl.gpio_ready = true;
	mutex_unlock(&pcl.status_lock);

	return 0;
}

static inline void _enable_clk(void) {
	union acpi_object *obj;
	union acpi_object obj_args[3], argv4;

	obj_args[0].integer.type = ACPI_TYPE_INTEGER;
	obj_args[0].integer.value = 0;
	obj_args[1].integer.type = ACPI_TYPE_INTEGER;
	obj_args[1].integer.value = 1;
	obj_args[2].integer.type = ACPI_TYPE_INTEGER;
	obj_args[2].integer.value = 1;

	argv4.type = ACPI_TYPE_PACKAGE;
	argv4.package.count = 3;
	argv4.package.elements = obj_args;

	obj = acpi_evaluate_dsm(pcl.clk_handle, &int3472_clk_guid, 0, 0x1, &argv4);
}

// debug
static inline void _gpiod_set_value_cansleep(struct gpio_desc *desc, int value) {
	printk("DEBUG|%s|%d| SET GPIO [desc:0x%p] %s", __FUNCTION__, __LINE__, desc, value ? "ON":"OFF");
	gpiod_set_value_cansleep(desc, value);
}

static void power_ctrl_logic_remove(struct pci_dev *pdev)
{
	dev_dbg(&pdev->dev, "@%s, enter\n", __func__);
	mutex_lock(&pcl.status_lock);
	pcl.gpio_ready = false;
	_gpiod_set_value_cansleep(pcl.reset_gpio, 0);
	gpiod_put(pcl.reset_gpio);
	_gpiod_set_value_cansleep(pcl.powerdn_gpio, 0);
	gpiod_put(pcl.powerdn_gpio);
	_gpiod_set_value_cansleep(pcl.indled_gpio, 0);
	gpiod_put(pcl.indled_gpio);
	mutex_unlock(&pcl.status_lock);
	dev_dbg(&pdev->dev, "@%s, exit\n", __func__);
}

static struct pci_device_id power_ctrl_logic_ids[] = {
	{ PCI_DEVICE(PCL_PCI_BRG_VEN_ID, PCL_PCI_BRG_PDT_ID) },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, power_ctrl_logic_ids);

static struct pci_driver power_ctrl_logic_driver = {
	.name     = PCL_DRV_NAME,
	.id_table = power_ctrl_logic_ids,
	.probe    = power_ctrl_logic_probe,
	.remove   = power_ctrl_logic_remove,
};

static int __init power_ctrl_logic_init(void)
{
	mutex_init(&pcl.status_lock);
	return pci_register_driver(&power_ctrl_logic_driver);
}

static void __exit power_ctrl_logic_exit(void)
{
	pci_unregister_driver(&power_ctrl_logic_driver);
}
module_init(power_ctrl_logic_init);
module_exit(power_ctrl_logic_exit);

int power_ctrl_logic_set_power(int on)
{
	mutex_lock(&pcl.status_lock);
	if (!pcl.gpio_ready || on < 0 || on > 1) {
		pr_debug("@%s,failed to set power, gpio_ready=%d, on=%d\n",
			 __func__, pcl.gpio_ready, on);
		mutex_unlock(&pcl.status_lock);
		return -EBUSY;
	}
	if (pcl.power_on != on) {
		if (on) {
			_gpiod_set_value_cansleep(pcl.reset_gpio, 1);
			_enable_clk();
			_gpiod_set_value_cansleep(pcl.powerdn_gpio, 1);
			_gpiod_set_value_cansleep(pcl.indled_gpio, 1);
			usleep_range(1000, 2000);
			_gpiod_set_value_cansleep(pcl.reset_gpio, 1);
			usleep_range(1500, 1800);
		} else {
			_gpiod_set_value_cansleep(pcl.indled_gpio, 0);
			_gpiod_set_value_cansleep(pcl.reset_gpio, 0);
			_gpiod_set_value_cansleep(pcl.powerdn_gpio, 0);
		}
	pcl.power_on = on;
	}

	mutex_unlock(&pcl.status_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(power_ctrl_logic_set_power);

MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Qiu, Tianshu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Xu, Chongyang <chongyang.xu@intel.com>");
MODULE_DESCRIPTION("Power Control Logic Driver");
MODULE_LICENSE("GPL v2");
