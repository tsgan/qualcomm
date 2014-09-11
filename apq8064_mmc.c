/*-
 * Copyright (c) 2014 Ganbold Tsagaankhuu <ganbold@freebsd.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>

#include <sys/kdb.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include <arm/qualcomm/apq8064_mmc.h>

#define	DEBUG	1

#ifdef DEBUG
#define debugf(fmt, args...) do { printf("%s(): ", __func__);   \
    printf(fmt,##args); } while (0)
#else
#define debugf(fmt, args...)
#endif

struct apq8064_mmc_dmamap_arg {
	bus_addr_t		apq_dma_busaddr;
};

struct apq8064_mmc_softc {
	device_t		apq_dev;
	struct mtx		apq_mtx;
	struct resource *	apq_mem_res;
	struct resource *	apq_irq_res;
	bus_space_tag_t		apq_bst;
	bus_space_handle_t	apq_bsh;
	void *			apq_intrhand;
	struct mmc_host		apq_host;
	struct mmc_request *	apq_req;
	struct mmc_data *	apq_data;
	uint32_t		apq_flags;
#define	APQ8064_SD_FLAGS_IGNORECRC	(1 << 0)
	int			apq_xfer_direction;
#define	DIRECTION_READ		0
#define	DIRECTION_WRITE		1
	int			apq_xfer_done;
	int			apq_bus_busy;
	bus_dma_tag_t		apq_dma_tag;
	bus_dmamap_t		apq_dma_map;
	bus_addr_t		apq_buffer_phys;
	void *			apq_buffer;
};

#define	APQ8064_SD_MAX_BLOCKSIZE	4096

static int apq8064_mmc_probe(device_t);
static int apq8064_mmc_attach(device_t);
static int apq8064_mmc_detach(device_t);
static void apq8064_mmc_intr(void *);

static void apq8064_mmc_cmd(struct apq8064_mmc_softc *, struct mmc_command *);
static void apq8064_mmc_setup_xfer(struct apq8064_mmc_softc *, struct mmc_data *);
static void apq8064_mmc_xfer_done(struct apq8064_mmc_softc *sc);
static void apq8064_mmc_end_of_data(struct apq8064_mmc_softc *sc);

static int apq8064_mmc_update_ios(device_t, device_t);
static int apq8064_mmc_request(device_t, device_t, struct mmc_request *);
static int apq8064_mmc_get_ro(device_t, device_t);
static int apq8064_mmc_acquire_host(device_t, device_t);
static int apq8064_mmc_release_host(device_t, device_t);

static void apq8064_mmc_dmamap_cb(void *, bus_dma_segment_t *, int, int);

#define	apq8064_mmc_lock(_sc)						\
    mtx_lock(&_sc->apq_mtx);
#define	apq8064_mmc_unlock(_sc)						\
    mtx_unlock(&_sc->apq_mtx);

#define	apq8064_mmc_read_4(_sc, _reg)					\
    bus_space_read_4(_sc->apq_bst, _sc->apq_bsh, _reg)

//#define	apq8064_mmc_write_4(_sc, _reg, _value)				\
//    bus_space_write_4(_sc->apq_bst, _sc->apq_bsh, _reg, _value)

static void
apq8064_mmc_write_4(struct apq8064_mmc_softc *sc, uint32_t reg, uint32_t value)
{

	bus_space_write_4(sc->apq_bst, sc->apq_bsh, reg, value);
	/* 3 clk delay required here! */
	/*
	 * Writes to MCI port are not effective for 3 ticks of PCLK.
	 * The min pclk is 144KHz which gives 6.94 us/tick.
	 * Thus 21us == 3 ticks.
	 */
//	DELAY(21);
	DELAY(100);
}

static int
apq8064_mmc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "qcom,mmc"))
		return (ENXIO);

	device_set_desc(dev, "Qualcomm MMC/SD controller");
	return (BUS_PROBE_DEFAULT);
}

static int
apq8064_mmc_attach(device_t dev)
{
	struct apq8064_mmc_softc *sc = device_get_softc(dev);
	struct apq8064_mmc_dmamap_arg ctx;
	device_t child;
	int rid, err;

	sc->apq_dev = dev;
	sc->apq_req = NULL;

	mtx_init(&sc->apq_mtx, "apq8064_mmc", "apq_mmc", MTX_DEF);

	rid = 0;
	sc->apq_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->apq_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	sc->apq_bst = rman_get_bustag(sc->apq_mem_res);
	sc->apq_bsh = rman_get_bushandle(sc->apq_mem_res);

	debugf("virtual register space: 0x%08lx\n", sc->apq_bsh);

	rid = 0;
	sc->apq_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (!sc->apq_irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->apq_mem_res);
		return (ENXIO);
	}

	if (bus_setup_intr(dev, sc->apq_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, apq8064_mmc_intr, sc, &sc->apq_intrhand))
	{
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->apq_mem_res);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->apq_irq_res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	sc->apq_host.f_min = 144000;
	sc->apq_host.f_max = 50000000;
	sc->apq_host.host_ocr = MMC_OCR_300_310 | MMC_OCR_310_320 |
	    MMC_OCR_320_330 | MMC_OCR_330_340;

	sc->apq_host.caps = MMC_CAP_4_BIT_DATA;

	sc->apq_host.caps |= MMC_CAP_HSPEED;

	/* Alloc DMA memory */
	err = bus_dma_tag_create(
	    bus_get_dma_tag(sc->apq_dev),
	    4, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    APQ8064_SD_MAX_BLOCKSIZE, 1,	/* maxsize, nsegments */
	    APQ8064_SD_MAX_BLOCKSIZE, 0,	/* maxsegsize, flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->apq_dma_tag);

	err = bus_dmamem_alloc(sc->apq_dma_tag, (void **)&sc->apq_buffer,
	    0, &sc->apq_dma_map);
	if (err) {
		device_printf(dev, "cannot allocate framebuffer\n");
		goto fail;
	}

	err = bus_dmamap_load(sc->apq_dma_tag, sc->apq_dma_map, sc->apq_buffer,
	    APQ8064_SD_MAX_BLOCKSIZE, apq8064_mmc_dmamap_cb, &ctx, BUS_DMA_NOWAIT);
	if (err) {
		device_printf(dev, "cannot load DMA map\n");
		goto fail;
	}

	sc->apq_buffer_phys = ctx.apq_dma_busaddr;

	child = device_add_child(dev, "mmc", -1);
	if (!child) {
		device_printf(dev, "attaching MMC bus failed!\n");
		bus_teardown_intr(dev, sc->apq_irq_res, sc->apq_intrhand);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->apq_mem_res);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->apq_irq_res);
		return (ENXIO);
	}

	bus_generic_probe(dev);
	bus_generic_attach(dev);

//	apq8064_mmc_write_4(sc, APQ8064_SD_POWER, APQ8064_SD_POWER_CTRL_ON);

	apq8064_mmc_write_4(sc, APQ8064_SD_MASK0, 0);
	apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, 0x5e007ff);
	apq8064_mmc_write_4(sc, APQ8064_SD_MASK0, APQ8064_SD_IRQENABLE);

	return (0);

fail:
	mtx_destroy(&sc->apq_mtx);
	if (sc->apq_intrhand)
		bus_teardown_intr(dev, sc->apq_irq_res, sc->apq_intrhand);
	if (sc->apq_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->apq_irq_res);
	if (sc->apq_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->apq_mem_res);
	return (err);
}

static int
apq8064_mmc_detach(device_t dev)
{

	return (EBUSY);
}

static void
apq8064_mmc_intr(void *arg)
{
	struct apq8064_mmc_softc *sc = (struct apq8064_mmc_softc *)arg;
	struct mmc_command *cmd;
	uint32_t status;

	status = apq8064_mmc_read_4(sc, APQ8064_SD_STATUS);
	status &= apq8064_mmc_read_4(sc, APQ8064_SD_MASK0);
	apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, status);

	if (status != 0x000c0000)
		debugf("interrupt: 0x%08x\n", status);

	if (status & APQ8064_SD_STATUS_CMDCRCFAIL) {
		cmd = sc->apq_req->cmd;
		cmd->error = sc->apq_flags & APQ8064_SD_FLAGS_IGNORECRC
		    ? MMC_ERR_NONE : MMC_ERR_BADCRC;
		cmd->resp[0] = apq8064_mmc_read_4(sc, APQ8064_SD_RESP0);
		sc->apq_req->done(sc->apq_req);
		sc->apq_req = NULL;
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_CMDCRCFAIL);
	}

	if (status & APQ8064_SD_STATUS_CMDACTIVE)
	{
		debugf("command active\n");
		cmd = sc->apq_req->cmd;
		cmd->resp[0] = apq8064_mmc_read_4(sc, APQ8064_SD_RESP0);
		sc->apq_req->done(sc->apq_req);
		sc->apq_req = NULL;
	}
	
	if (status & APQ8064_SD_STATUS_DATATIMEOUT) {
		device_printf(sc->apq_dev, "data timeout\n");
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_DATATIMEOUT);
	}

	if (status & APQ8064_SD_STATUS_TXUNDERRUN) {
		device_printf(sc->apq_dev, "TX underrun\n");
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_TXUNDERRUN);
	}
	
	if (status & APQ8064_SD_STATUS_CMDRESPEND) {
		debugf("command response\n");
		cmd = sc->apq_req->cmd;
		
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[3] = apq8064_mmc_read_4(sc, APQ8064_SD_RESP3);
			cmd->resp[2] = apq8064_mmc_read_4(sc, APQ8064_SD_RESP2);
			cmd->resp[1] = apq8064_mmc_read_4(sc, APQ8064_SD_RESP1);
		}

		cmd->resp[0] = apq8064_mmc_read_4(sc, APQ8064_SD_RESP0);
		cmd->error = MMC_ERR_NONE;
	
		if (cmd->data && (cmd->data->flags & MMC_DATA_WRITE)) {
			apq8064_mmc_setup_xfer(sc, sc->apq_req->cmd->data);
			apq8064_mmc_xfer_done(sc);
		}
		if (!cmd->data) {
			sc->apq_req->done(sc->apq_req);
			sc->apq_req = NULL;
		}

		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_CMDRESPEND);
	}

	if (status & APQ8064_SD_STATUS_CMDSENT) {
		debugf("command sent\n");
		cmd = sc->apq_req->cmd;
		cmd->error = MMC_ERR_NONE;
		sc->apq_req->done(sc->apq_req);
		sc->apq_req = NULL;
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_CMDSENT);
	}
	
	if (status & APQ8064_SD_STATUS_DATAEND) {
		apq8064_mmc_end_of_data(sc);
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_DATAEND);
	}

	if (status & APQ8064_SD_STATUS_CMDTIMEOUT) {
		device_printf(sc->apq_dev, "command response timeout\n");
		cmd = sc->apq_req->cmd;
		cmd->error = MMC_ERR_TIMEOUT;
		sc->apq_req->done(sc->apq_req);
		sc->apq_req = NULL;
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_CMDTIMEOUT);
		return;
	}

	if (status & APQ8064_SD_STATUS_STARTBITERR) {
		device_printf(sc->apq_dev, "start bit error\n");
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_STARTBITERR);
	}

	if (status & APQ8064_SD_STATUS_DATACRCFAIL) {		
		device_printf(sc->apq_dev, "data CRC error\n");
		debugf("data buffer: %p\n", sc->apq_buffer);
		cmd = sc->apq_req->cmd;
		cmd->error = MMC_ERR_BADCRC;
		sc->apq_req->done(sc->apq_req);
		sc->apq_req = NULL;

		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_DATACRCFAIL);
	}

	if (status & APQ8064_SD_STATUS_DATABLOCKEND) {
		debugf("data block end\n");
		if (sc->apq_xfer_direction == DIRECTION_READ)
			memcpy(sc->apq_data->data, sc->apq_buffer, sc->apq_data->len);

		if (sc->apq_xfer_direction == DIRECTION_WRITE) {
			apq8064_mmc_write_4(sc, APQ8064_SD_DATACTRL, 0);
		}
	
		sc->apq_req->done(sc->apq_req);
		sc->apq_req = NULL;
		apq8064_mmc_write_4(sc, APQ8064_SD_CLEAR, APQ8064_SD_STATUS_DATABLOCKEND);
	}

//	debugf("done\n");
}

static void
apq8064_mmc_end_of_data(struct apq8064_mmc_softc *sc)
{

	apq8064_mmc_xfer_done(sc);
	if (sc->apq_req) {
		sc->apq_req->cmd->error = MMC_ERR_NONE;
		sc->apq_req->done(sc->apq_req);
		sc->apq_req = NULL;
	}
}


static int
apq8064_mmc_request(device_t bus, device_t child, struct mmc_request *req)
{
	struct apq8064_mmc_softc *sc = device_get_softc(bus);

	debugf("request: %p\n", req);

	apq8064_mmc_lock(sc);
	if (sc->apq_req)
		return (EBUSY);

	sc->apq_req = req;

	if (req->cmd->data && req->cmd->data->flags & MMC_DATA_WRITE) {
		memcpy(sc->apq_buffer, req->cmd->data->data, req->cmd->data->len);
		apq8064_mmc_cmd(sc, req->cmd);
		apq8064_mmc_unlock(sc);
		return (0);
	}

	if (req->cmd->data) {
		apq8064_mmc_setup_xfer(sc, req->cmd->data);
		apq8064_mmc_xfer_done(sc);
	}

	apq8064_mmc_cmd(sc, req->cmd);
	apq8064_mmc_unlock(sc);

	return (0);
}

static void
apq8064_mmc_cmd(struct apq8064_mmc_softc *sc, struct mmc_command *cmd)
{
	uint32_t cmdreg = 0;

	if (apq8064_mmc_read_4(sc, APQ8064_SD_COMMAND) & APQ8064_SD_COMMAND_ENABLE) {
		apq8064_mmc_write_4(sc, APQ8064_SD_COMMAND, 0);
		DELAY(1000);
	}

	sc->apq_flags &= ~APQ8064_SD_FLAGS_IGNORECRC;

	cmdreg |= (cmd->opcode | APQ8064_SD_COMMAND_ENABLE);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			cmdreg |= APQ8064_SD_COMMAND_LONGRSP;
		cmdreg |= APQ8064_SD_COMMAND_RESPONSE;
	}

	if (MMC_RSP(cmd->flags) == MMC_RSP_R2)
		cmdreg |= APQ8064_SD_COMMAND_LONGRSP;

	if (MMC_RSP(cmd->flags) == MMC_RSP_R3)
		sc->apq_flags |= APQ8064_SD_FLAGS_IGNORECRC;

//	cmdreg |= APQ8064_SD_COMMAND_ENABLE;
	cmdreg |= (cmd->opcode & APQ8064_SD_COMMAND_CMDINDEXMASK);

	if ((((cmd->opcode == 17) || (cmd->opcode == 18))  ||
	     ((cmd->opcode == 24) || (cmd->opcode == 25))) ||
	      (cmd->opcode == 53))
		cmdreg |= APQ8064_SD_COMMAND_DATCMD;

	debugf("cmd: %d arg: 0x%08x, cmdreg: 0x%08x\n", cmd->opcode, cmd->arg, cmdreg);

//	apq8064_mmc_write_4(sc, APQ8064_SD_MASK0, 0xffffffff);
//	apq8064_mmc_write_4(sc, APQ8064_SD_MASK1, 0xffffffff);
	apq8064_mmc_write_4(sc, APQ8064_SD_ARGUMENT, cmd->arg);
	apq8064_mmc_write_4(sc, APQ8064_SD_COMMAND, cmdreg);
}

static void
apq8064_mmc_xfer_done(struct apq8064_mmc_softc *sc)
{
	void *resp_ptr = sc->apq_req->cmd->data->data;

	if (sc->apq_xfer_direction == DIRECTION_WRITE) {
		memcpy(sc->apq_buffer, resp_ptr, sc->apq_req->cmd->data->len);
	}
	else {
		memcpy(resp_ptr, sc->apq_buffer, sc->apq_req->cmd->data->len);
	}

	return;
}

static void
apq8064_mmc_setup_xfer(struct apq8064_mmc_softc *sc, struct mmc_data *data)
{
	uint32_t datactrl = 0;

	sc->apq_data = data;
	sc->apq_xfer_done = 0;

	debugf("data: %p, len: %d, %s\n", data,
	    data->len, (data->flags & MMC_DATA_READ) ? "read" : "write");

	if (data->flags & MMC_DATA_READ) {
		sc->apq_xfer_direction = DIRECTION_READ;
	}

	if (data->flags & MMC_DATA_WRITE) {
		sc->apq_xfer_direction = DIRECTION_WRITE;
	}

	datactrl |= (sc->apq_xfer_direction 
	    ? APQ8064_SD_DATACTRL_WRITE 
	    : APQ8064_SD_DATACTRL_READ);

//	datactrl |= APQ8064_SD_DATACTRL_DMAENABLE | APQ8064_SD_DATACTRL_ENABLE;
	datactrl |= APQ8064_SD_DATACTRL_ENABLE;
	datactrl |= (ffs(data->len) - 1) << 4;

	debugf("datactrl: 0x%08x\n", datactrl);

	apq8064_mmc_write_4(sc, APQ8064_SD_DATATIMER, 0xFFFF0000);
	apq8064_mmc_write_4(sc, APQ8064_SD_DATALENGTH, data->len);
	apq8064_mmc_write_4(sc, APQ8064_SD_DATACTRL, datactrl);
}

static int
apq8064_mmc_read_ivar(device_t bus, device_t child, int which, 
    uintptr_t *result)
{
	struct apq8064_mmc_softc *sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = sc->apq_host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = sc->apq_host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = sc->apq_host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = sc->apq_host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = sc->apq_host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = sc->apq_host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = sc->apq_host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = sc->apq_host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = sc->apq_host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = sc->apq_host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = sc->apq_host.ios.vdd;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = sc->apq_host.caps;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*(int *)result = 1;
		break;
	}

	return (0);
}

static int
apq8064_mmc_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value)
{
	struct apq8064_mmc_softc *sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		sc->apq_host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->apq_host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->apq_host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->apq_host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->apq_host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->apq_host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->apq_host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->apq_host.ios.vdd = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
		return (EINVAL);
	}
	return (0);
}

static int
apq8064_mmc_update_ios(device_t bus, device_t child)
{
	struct apq8064_mmc_softc *sc = device_get_softc(bus);
	struct mmc_ios *ios = &sc->apq_host.ios;
	uint32_t clkdiv = 0, pwr = 0;

	if (ios->bus_width == bus_width_4) {
		debugf("using wide bus mode\n");
		clkdiv |= APQ8064_SD_CLOCK_WIDEBUS;
	}

//	if (ios->clock > 400000)
//		clk |= (1 << 9); /* PWRSAVE */

	clkdiv |= (1 << 12); /* FLOW_ENA */
	clkdiv |= (1 << 15); /* feedback clock */

	/* Calculate clock divider */
//	clkdiv = (APQ8064_SD_CLK / (2 * ios->clock)) - 1;

	/* Clock rate should not exceed rate requested in ios */
//	if ((APQ8064_SD_CLK / (2 * (clkdiv + 1))) > ios->clock)
//		clkdiv++;

	debugf("clock: %dHz, clkdiv: %d\n", ios->clock, clkdiv);

	apq8064_mmc_write_4(sc, APQ8064_SD_CLOCK, clkdiv | APQ8064_SD_CLOCK_ENABLE);

	switch (ios->power_mode) {
	case power_off:
		pwr |= APQ8064_SD_POWER_CTRL_OFF;
		break;
	case power_up:
		pwr |= APQ8064_SD_POWER_CTRL_UP;
		break;
	case power_on:
		pwr |= APQ8064_SD_POWER_CTRL_ON;
		break;
	}

	if (ios->bus_mode == opendrain)
		pwr |= APQ8064_SD_POWER_OPENDRAIN;

	apq8064_mmc_write_4(sc, APQ8064_SD_POWER, pwr);

	return (0);
}

static int
apq8064_mmc_get_ro(device_t bus, device_t child)
{

	return (0);
}

static int
apq8064_mmc_acquire_host(device_t bus, device_t child)
{
	struct apq8064_mmc_softc *sc = device_get_softc(bus);
	int error = 0;

	apq8064_mmc_lock(sc);
	while (sc->apq_bus_busy)
		error = mtx_sleep(sc, &sc->apq_mtx, PZERO, "mmcah", 0);

	sc->apq_bus_busy++;
	apq8064_mmc_unlock(sc);
	return (error);
}

static int
apq8064_mmc_release_host(device_t bus, device_t child)
{
	struct apq8064_mmc_softc *sc = device_get_softc(bus);

	apq8064_mmc_lock(sc);
	sc->apq_bus_busy--;
	wakeup(sc);
	apq8064_mmc_unlock(sc);
	return (0);
}

static void
apq8064_mmc_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int err)
{
	struct apq8064_mmc_dmamap_arg *ctx;

	if (err)
		return;

	ctx = (struct apq8064_mmc_dmamap_arg *)arg;
	ctx->apq_dma_busaddr = segs[0].ds_addr;
}

static device_method_t apq8064_mmc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apq8064_mmc_probe),
	DEVMETHOD(device_attach,	apq8064_mmc_attach),
	DEVMETHOD(device_detach,	apq8064_mmc_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	apq8064_mmc_read_ivar),
	DEVMETHOD(bus_write_ivar,	apq8064_mmc_write_ivar),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	apq8064_mmc_update_ios),
	DEVMETHOD(mmcbr_request,	apq8064_mmc_request),
	DEVMETHOD(mmcbr_get_ro,		apq8064_mmc_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	apq8064_mmc_acquire_host),
	DEVMETHOD(mmcbr_release_host,	apq8064_mmc_release_host),

	{ 0, 0 }
};

static devclass_t apq8064_mmc_devclass;

static driver_t apq8064_mmc_driver = {
	"apq8064_mmc",
	apq8064_mmc_methods,
	sizeof(struct apq8064_mmc_softc),
};

DRIVER_MODULE(apq8064_mmc, simplebus, apq8064_mmc_driver, apq8064_mmc_devclass, 0, 0);
