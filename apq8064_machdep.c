/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@gmail.com>
 * All rights reserved.
 *
 * This code is derived from software written for Brini by Mark Brinicombe
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * from: FreeBSD: //depot/projects/arm/src/sys/arm/ti/ti_machdep.c
 */

#include "opt_ddb.h"
#include "opt_platform.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#define _ARM32_BUS_DMA_PRIVATE
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/armreg.h>
#include <machine/bus.h>
#include <machine/devmap.h>
#include <machine/machdep.h>
#include <machine/platform.h> 

#include <dev/fdt/fdt_common.h>

vm_offset_t
platform_lastaddr(void)
{

        return (arm_devmap_lastaddr());
}

void
platform_probe_and_attach(void)
{

}

void
platform_gpio_init(void)
{
}

void
platform_late_init(void)
{

        /* Enable cache */
        cpufunc_control(CPU_CONTROL_DC_ENABLE|CPU_CONTROL_IC_ENABLE,
            CPU_CONTROL_DC_ENABLE|CPU_CONTROL_IC_ENABLE);
}


/*
 * Set up static device mappings.
 */
int
platform_devmap_init(void)
{

        arm_devmap_add_entry(0x16600000, 0x00100000);
        
        return (0);
}

struct arm32_dma_range *
bus_dma_get_range(void)
{

	return (NULL);
}

int
bus_dma_get_range_nb(void)
{

	return (0);
}

void
cpu_reset()
{

	printf("Reset failed!\n");
	while (1);
}

/*
 * Early putc routine for EARLY_PRINTF support.  To use, add to kernel config:
 *   option SOCDEV_PA=0x16600000
 *   option SOCDEV_VA=0x16600000
 *   option EARLY_PRINTF
 */
#if 0
static void
apq8064_early_putc(int c)
{
        volatile uint32_t * UART_STAT_REG = (uint32_t *)0x16640008;
        volatile uint32_t * UART_TX_REG   = (uint32_t *)0x16640070;
        volatile uint32_t * UART_TX_NCHAR_REG   = (uint32_t *)0x16640040;
        const uint32_t      UART_TXRDY    = (1 << 2);

        while ((*UART_STAT_REG & UART_TXRDY) == 0)
                continue;
        *UART_TX_NCHAR_REG = 1;
        *UART_TX_REG = c;
}
early_putc_t *early_putc = apq8064_early_putc;

#endif
