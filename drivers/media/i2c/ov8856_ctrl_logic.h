/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2020-2021 Intel Corporation. */

#ifndef __CTRL_LOGIC_H_
#define __CTRL_LOGIC_H_

#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/pci.h>

/* pci id for probe power control logic */
#define PCL_PCI_BRG_VEN_ID 0x8086
#define PCL_PCI_BRG_PDT_ID 0x9a12

#define GPIO_LED 19
#define GPIO_POWER 20
#define GPIO_RESET 22


/* XXX: Hardcoded ACPI path for gpio and clock resources.
 * This does only work on:
 * - Lenovo X12 Detachable Gen 1 (20UV000HMH) */
#define PATH_GPI0 "\\_SB.GPI0"
#define PATH_DSC0 "\\_SB.PC00.DSC0"

struct ov8856_ctrl_logic {
	/* gpio resource*/
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	struct gpio_desc *indled_gpio;
	/* clk resource*/
	acpi_handle clk_handle;
	/* status */
	struct mutex status_lock;
	bool power_on;
	bool gpio_ready;
};

/* exported function for extern module */
void ov8856_power(int on);
void ov8856_reset(int on);
void ov8856_clk(int on);
void ov8856_led(int on);
#endif
