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

/*
 * Qualcomm EHCI attachment driver for the USB Enhanced Host Controller.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_bus.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>
#include <dev/ofw/ofw_bus.h> 
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/usb/usb.h> 
#include <dev/usb/usbdi.h>

#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_util.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/ehci.h>
#include <dev/usb/controller/ehcireg.h>

#include "hsusb.h"

#define TLMM_BASE_ADDR      0x00800000

#define GPIO_CONFIG_ADDR(x) (0x1000 + (x)*0x10)
#define GPIO_IN_OUT_ADDR(x) (0x1004 + (x)*0x10)

struct qcom_ehci_softc {
        ehci_softc_t            base;
        struct resource         *res[3]; 
        bus_space_tag_t         gpio_bst;
        bus_space_handle_t      gpio_bsh;

};

static struct resource_spec qcom_ehci_spec[] = {
        { SYS_RES_MEMORY,       0,      RF_ACTIVE },
        { SYS_RES_MEMORY,       1,      RF_ACTIVE },
        { SYS_RES_IRQ,          0,      RF_ACTIVE },
        { -1, 0 }
};

#define EHCI_HC_DEVSTR			"Qualcomm Integrated USB 2.0 controller"

#define READ_4(sc, reg)		\
	bus_space_read_4((sc)->sc_io_tag, (sc)->sc_io_hdl, reg)

#define WRITE_4(sc, reg, data)	\
	bus_space_write_4((sc)->sc_io_tag, (sc)->sc_io_hdl, reg, data)

static device_attach_t qcom_ehci_attach;
static device_detach_t qcom_ehci_detach;

bs_r_1_proto(reversed);
bs_w_1_proto(reversed);

/* GPIO TLMM: Direction */
#define QGPIO_INPUT      0
#define QGPIO_OUTPUT     1

/* GPIO TLMM: Pullup/Pulldown */ 
#define QGPIO_NO_PULL    0
#define QGPIO_PULL_DOWN  1
#define QGPIO_KEEPER     2
#define QGPIO_PULL_UP    3

/* GPIO TLMM: Drive Strength */
#define QGPIO_2MA        0
#define QGPIO_4MA        1
#define QGPIO_6MA        2
#define QGPIO_8MA        3
#define QGPIO_10MA       4
#define QGPIO_12MA       5
#define QGPIO_14MA       6
#define QGPIO_16MA       7

/* GPIO TLMM: Status */
#define QGPIO_ENABLE     1
#define QGPIO_DISABLE    0

void gpio_tlmm_config(struct qcom_ehci_softc *esc, uint32_t gpio, uint8_t func,
                      uint8_t dir, uint8_t pull,
                      uint8_t drvstr, uint32_t enable);
void gpio_set(struct qcom_ehci_softc *esc, uint32_t gpio, uint32_t dir);


void gpio_tlmm_config(struct qcom_ehci_softc *esc, uint32_t gpio, uint8_t func,
                      uint8_t dir, uint8_t pull,
                      uint8_t drvstr, uint32_t enable)
{
        unsigned int val = 0;

        val |= pull;
        val |= func << 2;
        val |= drvstr << 6;
        val |= enable << 9;
        bus_space_write_4(esc->gpio_bst, esc->gpio_bsh, GPIO_CONFIG_ADDR(gpio), val);
        return;
}

void gpio_set(struct qcom_ehci_softc *esc, uint32_t gpio, uint32_t dir)
{

        bus_space_write_4(esc->gpio_bst, esc->gpio_bsh, GPIO_IN_OUT_ADDR(gpio), dir);
        return;
}

static int
qcom_ehci_probe(device_t self)
{
	if (!ofw_bus_is_compatible(self, "qcom,usb-ehci")) 
		return (ENXIO);

	device_set_desc(self, EHCI_HC_DEVSTR);

	return (BUS_PROBE_DEFAULT);
}

static int
qcom_ehci_attach(device_t self)
{
	ehci_softc_t *sc;
        struct qcom_ehci_softc *esc;
	bus_space_handle_t bsh;
	int err;
	uint32_t val;

        esc = device_get_softc(self);
        sc = &esc->base;
	sc->sc_bus.parent = self;
	sc->sc_bus.devices = sc->sc_devices;
	sc->sc_bus.devices_max = EHCI_MAX_DEVICES;

	/* get all DMA memory */
	if (usb_bus_mem_alloc_all(&sc->sc_bus,
	    USB_GET_DMA_TAG(self), &ehci_iterate_hw_softc)) {
		return (ENOMEM);
	}

	sc->sc_bus.usbrev = USB_REV_2_0;

        if (bus_alloc_resources(self, qcom_ehci_spec, esc->res)) {
                device_printf(self, "could not allocate resources\n");
                return (ENXIO);
        }
        /* EHCI registers */
        sc->sc_io_tag = rman_get_bustag(esc->res[0]);
        bsh = rman_get_bushandle(esc->res[0]);
        sc->sc_io_size = rman_get_size(esc->res[0]);

        /* GPIO */
        esc->gpio_bst = rman_get_bustag(esc->res[1]);
        esc->gpio_bsh = rman_get_bushandle(esc->res[1]);

	if (bus_space_subregion(sc->sc_io_tag, bsh, 0x100,
	    sc->sc_io_size, &sc->sc_io_hdl) != 0)
		panic("%s: unable to subregion USB host registers",
		    device_get_name(self));

	sc->sc_bus.bdev = device_add_child(self, "usbus", -1);
	if (!sc->sc_bus.bdev) {
		device_printf(self, "Could not add USB device\n");
		goto error;
	}
	device_set_ivars(sc->sc_bus.bdev, &sc->sc_bus);
	device_set_desc(sc->sc_bus.bdev, EHCI_HC_DEVSTR);

	sprintf(sc->sc_vendor, "Qualcomm");

	err = bus_setup_intr(self, esc->res[2], INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, (driver_intr_t *)ehci_interrupt, sc, &sc->sc_intr_hdl);
	if (err) {
		device_printf(self, "Could not setup irq, %d\n", err);
		sc->sc_intr_hdl = NULL;
		goto error;
	}

	sc->sc_flags |= EHCI_SCFLG_DONTRESET;

        /* Configure GPIO for output */ 
        gpio_tlmm_config(esc, 77, 0, QGPIO_OUTPUT, QGPIO_NO_PULL, QGPIO_8MA, QGPIO_ENABLE);

        /* Output High */
        gpio_set(esc, 77, 2);

	/* ehci phy reset */
	val = READ_4(sc, USB_PORTSC) & ~PORTSC_PTS_MASK;
	WRITE_4(sc, USB_PORTSC, val | PORTSC_PTS_ULPI);
	WRITE_4(sc, USB_USBCMD, USBCMD_RESET);

	/* reset */
	WRITE_4(sc, USB_USBCMD, 0x00080000);
	/* select ULPI phy */
	WRITE_4(sc, USB_PORTSC, 0x80000000);

	/* burst of unspecified len */
	WRITE_4(sc, USB_AHB_BURST, 0);
	/* HPROT mode */
	WRITE_4(sc, USB_AHB_MODE, 0x08);

	/* disable BAM */
	WRITE_4(sc, USB_GENCONFIG, (1 << 13));

//	READ_4(sc, USB_DCCPARAMS);
	/* Disable streaming mode and select host mode */
	WRITE_4(sc, USB_USBMODE, 0x13);

	/* go to RUN mode (D+ pullup enable) */
	WRITE_4(sc, USB_USBCMD, 0x00080000);

//	breakpoint();
	err = ehci_init(sc);
	if (!err)
		err = device_probe_and_attach(sc->sc_bus.bdev);

	if (err) {
		device_printf(self, "USB init failed err=%d\n", err);
		goto error;
	}
	return (0);

error:
	qcom_ehci_detach(self);
	return (ENXIO);
}

static int
qcom_ehci_detach(device_t self)
{
	ehci_softc_t *sc = device_get_softc(self);
	device_t bdev;
	int err;

	if (sc->sc_bus.bdev) {
		bdev = sc->sc_bus.bdev;
		device_detach(bdev);
		device_delete_child(self, bdev);
	}
	/* during module unload there are lots of children leftover */
	device_delete_children(self);

	if (sc->sc_irq_res && sc->sc_intr_hdl) {
		/*
		 * only call ehci_detach() after ehci_init()
		 */
		ehci_detach(sc);

		err = bus_teardown_intr(self, sc->sc_irq_res, sc->sc_intr_hdl);

		if (err)
			/* XXX or should we panic? */
			device_printf(self, "Could not tear down irq, %d\n",
			    err);
		sc->sc_intr_hdl = NULL;
	}

	if (sc->sc_irq_res) {
		bus_release_resource(self, SYS_RES_IRQ, 0, sc->sc_irq_res);
		sc->sc_irq_res = NULL;
	}
	if (sc->sc_io_res) {
		bus_release_resource(self, SYS_RES_MEMORY, 0,
		    sc->sc_io_res);
		sc->sc_io_res = NULL;
	}
	usb_bus_mem_free_all(&sc->sc_bus, &ehci_iterate_hw_softc);

	return (0);
}

static device_method_t ehci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, qcom_ehci_probe),
	DEVMETHOD(device_attach, qcom_ehci_attach),
	DEVMETHOD(device_detach, qcom_ehci_detach),
	DEVMETHOD(device_suspend, bus_generic_suspend),
	DEVMETHOD(device_resume, bus_generic_resume),
	DEVMETHOD(device_shutdown, bus_generic_shutdown),

	DEVMETHOD_END
};

static driver_t ehci_driver = {
	.name = "ehci",
	.methods = ehci_methods,
	.size = sizeof(ehci_softc_t),
};

static devclass_t ehci_devclass;

DRIVER_MODULE(ehci, simplebus, ehci_driver, ehci_devclass, 0, 0);
MODULE_DEPEND(ehci, usb, 1, 1, 1);
