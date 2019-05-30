/*
 * Copyright 2017-2018 NXP
 * Copyright 2019 Arm
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <assert.h>
#include <common/bl_common.h>
#include <drivers/console.h>
#include <common/debug.h>
#include <context.h>
#include <stdbool.h>
#include <drivers/generic_delay_timer.h>
#include <lib/mmio.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include <imx8mm_private.h>
#include <tbbr_img_def.h>
#include <common/desc_image_load.h>
#include <lib/optee_utils.h>

#include <imx_uart.h>

static void imx8mm_aips_config(void)
{
	/* config the AIPSTZ1 */
	mmio_write_32(0x301f0000, 0x77777777);
	mmio_write_32(0x301f0004, 0x77777777);
	mmio_write_32(0x301f0040, 0x0);
	mmio_write_32(0x301f0044, 0x0);
	mmio_write_32(0x301f0048, 0x0);
	mmio_write_32(0x301f004c, 0x0);
	mmio_write_32(0x301f0050, 0x0);

	/* config the AIPSTZ2 */
	mmio_write_32(0x305f0000, 0x77777777);
	mmio_write_32(0x305f0004, 0x77777777);
	mmio_write_32(0x305f0040, 0x0);
	mmio_write_32(0x305f0044, 0x0);
	mmio_write_32(0x305f0048, 0x0);
	mmio_write_32(0x305f004c, 0x0);
	mmio_write_32(0x305f0050, 0x0);

	/* config the AIPSTZ3 */
	mmio_write_32(0x309f0000, 0x77777777);
	mmio_write_32(0x309f0004, 0x77777777);
	mmio_write_32(0x309f0040, 0x0);
	mmio_write_32(0x309f0044, 0x0);
	mmio_write_32(0x309f0048, 0x0);
	mmio_write_32(0x309f004c, 0x0);
	mmio_write_32(0x309f0050, 0x0);

	/* config the AIPSTZ4 */
	mmio_write_32(0x32df0000, 0x77777777);
	mmio_write_32(0x32df0004, 0x77777777);
	mmio_write_32(0x32df0040, 0x0);
	mmio_write_32(0x32df0044, 0x0);
	mmio_write_32(0x32df0048, 0x0);
	mmio_write_32(0x32df004c, 0x0);
	mmio_write_32(0x32df0050, 0x0);
}

void bl2_el3_early_platform_setup(u_register_t arg1, u_register_t arg2,
				  u_register_t arg3, u_register_t arg4)
{
	int i;
	static console_uart_t console;

	/* enable CSU NS access permission */
	for (i = 0; i < 64; i++) {
		mmio_write_32(0x303e0000 + i * 4, 0x00ff00ff);
	}

	/* config the aips access permission */
	imx8mm_aips_config();

	console_imx_uart_register(IMX_BOOT_UART_BASE, IMX_BOOT_UART_CLK_IN_HZ,
		IMX_CONSOLE_BAUDRATE, &console);

	/* Open handles to a FIP image */
	plat_imx8mm_io_setup();

}

void bl2_el3_plat_arch_setup(void)
{
}

void bl2_platform_setup(void)
{
	generic_delay_timer_init();

	/* select the CKIL source to 32K OSC */
	mmio_write_32(0x30360124, 0x1);
}

int bl2_plat_handle_post_image_load(unsigned int image_id)
{
	int err = 0;
	bl_mem_params_node_t *bl_mem_params = get_bl_mem_params_node(image_id);
	bl_mem_params_node_t *pager_mem_params = NULL;
	bl_mem_params_node_t *paged_mem_params = NULL;

	assert(bl_mem_params);

	switch (image_id) {
	case BL32_IMAGE_ID:
		pager_mem_params = get_bl_mem_params_node(BL32_EXTRA1_IMAGE_ID);
		assert(pager_mem_params);

		paged_mem_params = get_bl_mem_params_node(BL32_EXTRA2_IMAGE_ID);
		assert(paged_mem_params);

		err = parse_optee_header(&bl_mem_params->ep_info,
					 &pager_mem_params->image_info,
					 &paged_mem_params->image_info);
		if (err != 0)
			WARN("OPTEE header parse error.\n");

		break;
	default:
		/* Do nothing in default case */
		break;
	}

	return err;
}

unsigned int plat_get_syscnt_freq2(void)
{
	return COUNTER_FREQUENCY;
}

void bl2_plat_runtime_setup(void)
{
	return;
}
