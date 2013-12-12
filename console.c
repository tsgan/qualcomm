/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@gmail.com>
 * All rights reserved.
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
 */

/* Simple UART DM console driver for Snapdragon */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/cons.h>
#include <sys/consio.h>
#include <sys/kernel.h>

#ifndef	APQ8064_UART_BASE
#define	APQ8064_UART_BASE		0xF6640000	/* UART0 */
#endif

#define	UART_DM_SR(base)		((base) + 0x008)
#define	UART_DM_SR_RXRDY		(1 << 0)
#define	UART_DM_SR_UART_OVERRUN		(1 << 4)

#define	UART_DM_SR_TXRDY		(1 << 2)
#define	UART_DM_SR_TXEMT		(1 << 3)
#define	UART_DM_RXSTALE			(1 << 3)
#define	UART_DM_TX_READY		(1 << 7)

#define	UART_DM_CR(base)		((base) + 0x10)
#define	UART_DM_CR_CH_CMD_LSB(x)	((x & 0x0f) << 4)
#define	UART_DM_CR_CH_CMD_MSB(x)	((x >> 4 ) << 11 )
#define	UART_DM_CR_CH_CMD(x)		\
    (UART_DM_CR_CH_CMD_LSB(x) | UART_DM_CR_CH_CMD_MSB(x))
#define	UART_DM_CMD_NULL		UART_DM_CR_CH_CMD(0)
#define	UART_DM_CMD_RESET_RX		UART_DM_CR_CH_CMD(1)
#define	UART_DM_CMD_RESET_TX		UART_DM_CR_CH_CMD(2)
#define	UART_DM_CMD_RESET_ERR_STAT	UART_DM_CR_CH_CMD(3)
#define	UART_DM_CMD_RES_STALE_INT	UART_DM_CR_CH_CMD(8)

/*UART General Command */
#define	UART_DM_CR_GENERAL_CMD(x)	((x) << 8)

#define	UART_DM_GCMD_NULL		UART_DM_CR_GENERAL_CMD(0)
#define	UART_DM_GCMD_CR_PROT_EN		UART_DM_CR_GENERAL_CMD(1)
#define	UART_DM_GCMD_CR_PROT_DIS	UART_DM_CR_GENERAL_CMD(2)
#define	UART_DM_GCMD_RES_TX_RDY_INT	UART_DM_CR_GENERAL_CMD(3)
#define	UART_DM_GCMD_SW_FORCE_STALE	UART_DM_CR_GENERAL_CMD(4)
#define	UART_DM_GCMD_ENA_STALE_EVT	UART_DM_CR_GENERAL_CMD(5)
#define	UART_DM_GCMD_DIS_STALE_EVT	UART_DM_CR_GENERAL_CMD(6)

/* Used for RX transfer initialization */
#define	UART_DM_DMRX(base)		((base) + 0x34)

/* Default DMRX value - any value bigger than FIFO size would be fine */
#define	UART_DM_DMRX_DEF_VALUE		0x220

#define	UART_DM_MISR(base)		((base) + 0x10)

/*
 * The base address of the uart registers.
 *
 * This is global so that it can be changed on the fly from the outside.
 * For instance, set apq8064_uart_base=physaddr and then call cninit()
 * as the first two lines of initarm() and enjoy printf() availability
 * through the tricky bits of startup.  After initarm() switches from
 * physical to virtual addressing, just set apq8064_uart_base=virtaddr
 * and printf keeps working.
 */
uint32_t apq8064_uart_base = APQ8064_UART_BASE;

/*
 * uart related funcs
 */
static uint32_t
uart_getreg(uint32_t *bas)
{

	return *((volatile uint32_t *)(bas));
}

static void
uart_setreg(uint32_t *bas, uint32_t val)
{

	*((volatile uint32_t *)(bas)) = val;
}

static int
ub_getc(void)
{
	int byte;
	static unsigned int word = 0;

	/* Check RXRDY status bit */
	while (!(uart_getreg((uint32_t *)UART_DM_SR(apq8064_uart_base)) &
	    UART_DM_SR_RXRDY))
		__asm __volatile("nop");

	/* Check Overrun error, reset Error Status when necessary */
	if (uart_getreg((uint32_t *)UART_DM_SR(apq8064_uart_base)) &
	    UART_DM_SR_UART_OVERRUN)
		uart_setreg((uint32_t *)UART_DM_CR(apq8064_uart_base),
		    UART_DM_CMD_RESET_ERR_STAT);

	/* Check if we've received stale event */
//	if (uart_getreg((uint32_t *)UART_DM_MISR(apq8064_uart_base)) &
//	    UART_DM_RXSTALE) {
//		/* Send command to reset stale interrupt */
//		uart_setreg((uint32_t *)UART_DM_CR(apq8064_uart_base),
//		    UART_DM_CMD_RES_STALE_INT);
//	}
	if (!word) {
		/*
		 * Read from FIFO only if it's a first read or all four
		 * characters of a word have been read.
		 */
		word = uart_getreg((uint32_t *)(apq8064_uart_base + 0x70));
	}
	byte = (int)word & 0xff;
	word = word >> 8;

	return byte;
}

static void
ub_putc(unsigned char c)
{
	if (c == '\n')
		ub_putc('\r');

	/* Check TX FIFO is empty. */
	if (!(uart_getreg((uint32_t *)UART_DM_SR(apq8064_uart_base)) &
	    UART_DM_SR_TXEMT)) {
		/* Wait until TX is ready */
		while (!(uart_getreg((uint32_t *)UART_DM_SR(apq8064_uart_base)) &
		    UART_DM_SR_TXRDY))
			__asm __volatile("nop");
	}

	/* Write number of char, 1 in this case */
	uart_setreg((uint32_t *)(apq8064_uart_base + 0x40), 1);

	/* Clear TX_READY interrupt */
//	uart_setreg((uint32_t *)UART_DM_CR(apq8064_uart_base),
//	    UART_DM_GCMD_RES_TX_RDY_INT);

	/* Wait till TX FIFO has space */
	while (!(uart_getreg((uint32_t *)UART_DM_SR(apq8064_uart_base)) &
	    UART_DM_SR_TXRDY))
		__asm __volatile("nop");

	/* Write char itself */
	uart_setreg((uint32_t *)(apq8064_uart_base + 0x70), c);
}

static cn_probe_t	uart_cnprobe;
static cn_init_t	uart_cninit;
static cn_term_t	uart_cnterm;
static cn_getc_t	uart_cngetc;
static cn_putc_t	uart_cnputc;
static cn_grab_t	uart_cngrab;
static cn_ungrab_t	uart_cnungrab;

static void
uart_cngrab(struct consdev *cp)
{
}

static void
uart_cnungrab(struct consdev *cp)
{
}


static void
uart_cnprobe(struct consdev *cp)
{

	sprintf(cp->cn_name, "uart");
	cp->cn_pri = CN_NORMAL;
}

static void
uart_cninit(struct consdev *cp)
{

	/* Init RX transfer */
	uart_setreg((uint32_t *)UART_DM_CR(apq8064_uart_base),
	    UART_DM_GCMD_DIS_STALE_EVT);
	uart_setreg((uint32_t *)UART_DM_CR(apq8064_uart_base),
	    UART_DM_CMD_RES_STALE_INT);
	uart_setreg((uint32_t *)UART_DM_DMRX(apq8064_uart_base),
	    UART_DM_DMRX_DEF_VALUE);
	uart_setreg((uint32_t *)UART_DM_CR(apq8064_uart_base),
	    UART_DM_GCMD_ENA_STALE_EVT);
}

void
uart_cnputc(struct consdev *cp, int c)
{

	ub_putc(c);
}

int
uart_cngetc(struct consdev * cp)
{

	return ub_getc();
}

static void
uart_cnterm(struct consdev * cp)
{
}

CONSOLE_DRIVER(uart);
