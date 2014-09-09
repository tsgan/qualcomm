/*-
 * Copyright (c) 2014 Ganbold Tsagaankhuu <ganbold@gmail.com>
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

#ifndef	_UART_DM_H_
#define	_UART_DM_H_

#define	UART_DM_EXTR_BITS(value, start_pos, end_pos) \
	    ((value << (32 - end_pos)) >> (32 - (end_pos - start_pos)))

/* UART Parity Mode */
enum UART_DM_PARITY_MODE {
	UART_DM_NO_PARITY,
	UART_DM_ODD_PARITY,
	UART_DM_EVEN_PARITY,
	UART_DM_SPACE_PARITY
};

/* UART Stop Bit Length */
enum UART_DM_STOP_BIT_LEN {
	UART_DM_SBL_9_16,
	UART_DM_SBL_1,
	UART_DM_SBL_1_9_16,
	UART_DM_SBL_2
};

/* UART Bits per Char */
enum UART_DM_BITS_PER_CHAR {
	UART_DM_5_BPS,
	UART_DM_6_BPS,
	UART_DM_7_BPS,
	UART_DM_8_BPS
};

/* 8-N-1 Configuration */
#define	UART_DM_8_N_1_MODE		(UART_DM_NO_PARITY | \
					(UART_DM_SBL_1 << 2) | \
					(UART_DM_8_BPS << 4))

/* UART_DM Registers */

/* UART Operational Mode Registers (HSUART) */
#define	UART_DM_MR1			(0x00)
#define	UART_DM_MR2			(0x04)

#define	UART_DM_RXBRK_ZERO_CHAR_OFF	(1 << 8)
#define	UART_DM_LOOPBACK		(1 << 7)

/* UART Clock Selection Register, write only */
#define	UART_DM_CSR			(0x08)
#define	UART_DM_CSR_115200		0xFF
#define	UART_DM_CSR_57600		0xEE
#define	UART_DM_CSR_38400		0xDD
#define	UART_DM_CSR_28800		0xCC
#define	UART_DM_CSR_19200		0xBB
#define	UART_DM_CSR_14400		0xAA
#define	UART_DM_CSR_9600		0x99
#define	UART_DM_CSR_7200		0x88
#define	UART_DM_CSR_4800		0x77
#define	UART_DM_CSR_3600		0x66
#define	UART_DM_CSR_2400		0x55
#define	UART_DM_CSR_1200		0x44
#define	UART_DM_CSR_600			0x33
#define	UART_DM_CSR_300			0x22
#define	UART_DM_CSR_150			0x11
#define	UART_DM_CSR_75			0x00

/* UART DM TX FIFO Registers - 4, write only */
#define	UART_DM_TF(x)			(0x70 + (4 * (x)))

/* UART Command Register, write only */
#define	UART_DM_CR			(0x10)
#define	 UART_DM_CR_RX_ENABLE		(1 << 0)
#define	 UART_DM_CR_RX_DISABLE		(1 << 1)
#define	 UART_DM_CR_TX_ENABLE		(1 << 2)
#define	 UART_DM_CR_TX_DISABLE		(1 << 3)

/* UART Channel Command */
#define	UART_DM_CR_CH_CMD_LSB(x)	((x & 0x0f) << 4)
#define	UART_DM_CR_CH_CMD_MSB(x)	((x >> 4 ) << 11 )
#define	UART_DM_CR_CH_CMD(x)		(UART_DM_CR_CH_CMD_LSB(x) | \
					UART_DM_CR_CH_CMD_MSB(x))

#define	UART_DM_CMD_NULL		UART_DM_CR_CH_CMD(0)
#define	UART_DM_CMD_RESET_RX		UART_DM_CR_CH_CMD(1)
#define	UART_DM_CMD_RESET_TX		UART_DM_CR_CH_CMD(2)
#define	UART_DM_CMD_RESET_ERR_STAT	UART_DM_CR_CH_CMD(3)
#define	UART_DM_CMD_RES_BRK_CHG_INT	UART_DM_CR_CH_CMD(4)
#define	UART_DM_CMD_START_BRK		UART_DM_CR_CH_CMD(5)
#define	UART_DM_CMD_STOP_BRK		UART_DM_CR_CH_CMD(6)
#define	UART_DM_CMD_RES_CTS_N		UART_DM_CR_CH_CMD(7)
#define	UART_DM_CMD_RES_STALE_INT	UART_DM_CR_CH_CMD(8)
#define	UART_DM_CMD_PACKET_MODE		UART_DM_CR_CH_CMD(9)
#define	UART_DM_CMD_MODE_RESET		UART_DM_CR_CH_CMD(C)
#define	UART_DM_CMD_SET_RFR_N		UART_DM_CR_CH_CMD(D)
#define	UART_DM_CMD_RES_RFR_N		UART_DM_CR_CH_CMD(E)
#define	UART_DM_CMD_RES_TX_ERR		UART_DM_CR_CH_CMD(10)
#define	UART_DM_CMD_CLR_TX_DONE		UART_DM_CR_CH_CMD(11)
#define	UART_DM_CMD_RES_BRKSTRT_INT	UART_DM_CR_CH_CMD(12)
#define	UART_DM_CMD_RES_BRKEND_INT	UART_DM_CR_CH_CMD(13)
#define	UART_DM_CMD_RES_PER_FRM_INT	UART_DM_CR_CH_CMD(14)

/* UART_DM_CR channel command bit value (register field is bits 8:4) */
#define	RESET_RX			0x10
#define	RESET_TX			0x20
#define	RESET_ERROR_STATUS		0x30
#define	RESET_BREAK_INT			0x40
#define	START_BREAK			0x50
#define	STOP_BREAK			0x60
#define	RESET_CTS			0x70
#define	RESET_STALE_INT			0x80
#define	RFR_LOW				0xD0
#define	RFR_HIGH			0xE0
#define	CR_PROTECTION_EN		0x100
#define	STALE_EVENT_ENABLE		0x500
#define	STALE_EVENT_DISABLE		0x600
#define	FORCE_STALE_EVENT		0x400
#define	CLEAR_TX_READY			0x300
#define	RESET_TX_ERROR			0x800
#define	RESET_TX_DONE			0x810

#define	UART_DM_MR1_AUTO_RFR_LEVEL1_BMSK	0xffffff00
#define	UART_DM_MR1_AUTO_RFR_LEVEL0_BMSK	0x3f
#define	UART_DM_MR1_CTS_CTL_BMSK		0x40
#define	UART_DM_MR1_RX_RDY_CTL_BMSK		0x80

#define	UART_DM_MR2_ERROR_MODE_BMSK		0x40
#define	UART_DM_MR2_BITS_PER_CHAR_BMSK		0x30

#define	UART_DM_MR2_STOP_BIT_LEN_BMSK		0xc

#define	UART_DM_MR2_PARITY_MODE_BMSK		0x3

#define	UART_DM_IPR_STALE_TIMEOUT_MSB_BMSK	0xffffff80
#define	UART_DM_IPR_STALE_LSB_BMSK		0x1f

/*UART General Command */
#define	UART_DM_GCMD_NULL		(0 << 8)
#define	UART_DM_GCMD_CR_PROT_EN		(1 << 8)
#define	UART_DM_GCMD_CR_PROT_DIS	(2 << 8)
#define	UART_DM_GCMD_RES_TX_RDY_INT	(3 << 8)
#define	UART_DM_GCMD_SW_FORCE_STALE	(4 << 8)
#define	UART_DM_GCMD_ENA_STALE_EVT	(5 << 8)
#define	UART_DM_GCMD_DIS_STALE_EVT	(6 << 8)

/* UART Interrupt Mask Register */
#define	UART_DM_IMR			(0x14)
/* these can be used for both ISR and IMR registers */
#define	UART_DM_TXLEV			(1 << 0)
#define	UART_DM_RXHUNT			(1 << 1)
#define	UART_DM_RXBRK_CHNG		(1 << 2)
#define	UART_DM_RXSTALE			(1 << 3)
#define	UART_DM_RXLEV			(1 << 4)
#define	UART_DM_DELTA_CTS		(1 << 5)
#define	UART_DM_CURRENT_CTS		(1 << 6)
#define	UART_DM_TX_READY		(1 << 7)
#define	UART_DM_TX_ERROR		(1 << 8)
#define	UART_DM_TX_DONE			(1 << 9)
#define	UART_DM_RXBREAK_START		(1 << 10)
#define	UART_DM_RXBREAK_END		(1 << 11)
#define	UART_DM_PAR_FRAME_ERR_IRQ	(1 << 12)

#define	UART_DM_IMR_ENABLED		(UART_DM_TX_READY | \
					UART_DM_TXLEV | \
					UART_DM_RXLEV | \
					UART_DM_RXSTALE)

/* UART Interrupt Programming Register */
#define	UART_DM_IPR			(0x18)
#define	UART_DM_STALE_TIMEOUT_LSB	(0x0f)
#define	UART_DM_STALE_TIMEOUT_MSB	0

/* UART Transmit/Receive FIFO Watermark Register */
#define	UART_DM_TFWR			(0x1c)

/* Interrupt is generated when FIFO level is less than or equal to this value */
#define	UART_DM_TFW_VALUE		0

#define	UART_DM_RFWR			(0x20)

/* Interrupt generated when no of words in RX FIFO is greater than this value */
#define	UART_DM_RFW_VALUE		0

/* UART Hunt Character Register */
#define	UART_DM_HCR			(0x24)

/* Used for RX transfer initialization */
#define	UART_DM_DMRX			(0x34)

/* Default DMRX value - any value bigger than FIFO size would be fine */
#define	UART_DM_DMRX_DEF_VALUE		(0x220)

/* Register to enable IRDA function */
#define	UART_DM_IRDA			(0x38)

/* UART Data Mover Enable Register */
#define	UART_DM_DMEN			(0x3c)

/* Number of characters for Transmission */
#define	UART_DM_NO_CHARS_FOR_TX		(0x40)

/* UART RX FIFO Base Address */
#define	UART_DM_BADR			(0x44)

/* UART Status Register */
#define	UART_DM_SR			(0x08)
/* register field mask mapping */
#define	 UART_DM_SR_RXRDY		(1 << 0)
#define	 UART_DM_SR_RXFULL		(1 << 1)
#define	 UART_DM_SR_TXRDY		(1 << 2)
#define	 UART_DM_SR_TXEMT		(1 << 3)
#define	 UART_DM_SR_UART_OVERRUN	(1 << 4)
#define	 UART_DM_SR_PAR_FRAME_ERR	(1 << 5)
#define	 UART_DM_RX_BREAK		(1 << 6)
#define	 UART_DM_HUNT_CHAR		(1 << 7)
#define	 UART_DM_RX_BRK_START_LAST	(1 << 8)

/* UART Receive FIFO Registers - 4 in numbers */
#define	UART_DM_RF(x)			(0x70 + (4 * (x)))

/* UART Masked Interrupt Status Register */
#define	UART_DM_MISR			(0x10)

/* UART Interrupt Status Register */
#define	UART_DM_ISR			(0x14)

/* Number of characters received since the end of last RX transfer */
#define	UART_DM_RX_TOTAL_SNAP		(0x38)

/* UART TX FIFO Status Register */
#define	UART_DM_TXFS			(0x4c)
#define	 UART_DM_TXFS_STATE_LSB(x)	UART_DM_EXTR_BITS(x,0,6)
#define	 UART_DM_TXFS_STATE_MSB(x)	UART_DM_EXTR_BITS(x,14,31)
#define	 UART_DM_TXFS_BUF_STATE(x)	UART_DM_EXTR_BITS(x,7,9)
#define	 UART_DM_TXFS_ASYNC_STATE(x)	UART_DM_EXTR_BITS(x,10,13)

/* UART RX FIFO Status Register */
#define	UART_DM_RXFS			(0x50)
#define	 UART_DM_RXFS_STATE_LSB(x)	UART_DM_EXTR_BITS(x,0,6)
#define	 UART_DM_RXFS_STATE_MSB(x)	UART_DM_EXTR_BITS(x,14,31)
#define	 UART_DM_RXFS_BUF_STATE(x)	UART_DM_EXTR_BITS(x,7,9)
#define	 UART_DM_RXFS_ASYNC_STATE(x)	UART_DM_EXTR_BITS(x,10,13)

#define	UART_DM_SIM_CFG_ADDR		0x80

#endif	/* _UART_DM_H_ */
