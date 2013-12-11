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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timeet.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>
#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/fdt.h>

#include <sys/kdb.h>

/**
 * Timer registers addr
 *
 */
#define	MSM_TMR_BASE		0xf200A000
#define	MSM_GPT_BASE		0x04
//#define	MSM_GPT1_BASE		0x14
#define	MSM_DGT_BASE		0x24
#define	SPSS_TIMER_STATUS	0x88

#define	GPT_REG(off)		(MSM_GPT_BASE + (off))
//#define	GPT1_REG(off)		(MSM_GPT1_BASE + (off))
#define	DGT_REG(off)		(MSM_DGT_BASE + (off))

#define	GPT_MATCH_VAL		GPT_REG(0x0000)
#define	GPT_COUNT_VAL		GPT_REG(0x0004)
#define	GPT_ENABLE		GPT_REG(0x0008)
#define	GPT_CLEAR		GPT_REG(0x000C)
/*
#define	GPT1_MATCH_VAL		GPT1_REG(0x0000)
#define	GPT1_COUNT_VAL		GPT1_REG(0x0004)
#define	GPT1_ENABLE		GPT1_REG(0x0008)
#define	GPT1_CLEAR		GPT1_REG(0x000C)
*/
#define	DGT_MATCH_VAL		DGT_REG(0x0000)
#define	DGT_COUNT_VAL		DGT_REG(0x0004)
#define	DGT_ENABLE		DGT_REG(0x0008)
#define	DGT_CLEAR		DGT_REG(0x000C)
#define	DGT_CLK_CTL		DGT_REG(0x0010)

#define	GPT_ENABLE_CLR_ON_MATCH_EN	2
#define	GPT_ENABLE_EN			1
#define	DGT_ENABLE_CLR_ON_MATCH_EN	2
#define	DGT_ENABLE_EN			1

#define	SPSS_TIMER_STATUS_DGT_EN	(1 << 0)

#define	SYS_TIMER_CLKSRC		32768 /* clock source */

struct apq8064_timer_softc {
	device_t 	sc_dev;
	struct resource *res[2];
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	void 		*sc_ih;		/* interrupt handler */
	uint32_t 	sc_period;
	uint32_t 	timer0_freq;
	struct eventtimer et;
};

int apq8064_timer_get_timerfreq(struct apq8064_timer_softc *);

#define timer_read_4(sc, reg)	\
	bus_space_read_4(sc->sc_bst, sc->sc_bsh, reg)
#define timer_write_4(sc, reg, val)	\
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, reg, val)

static u_int	apq8064_timer_get_timecount(struct timecounter *);
static int	apq8064_timer_timer_start(struct eventtimer *,
    sbintime_t first, sbintime_t period);
static int	apq8064_timer_timer_stop(struct eventtimer *);

static int apq8064_timer_initialized = 0;
static int apq8064_timer_hardclock(void *);
static int apq8064_timer_probe(device_t);
static int apq8064_timer_attach(device_t);

static struct timecounter apq8064_timer_timecounter = {
	.tc_name           = "apq8064_timer timer0",
	.tc_get_timecount  = apq8064_timer_get_timecount,
	.tc_counter_mask   = ~0u,
	.tc_frequency      = 0,
	.tc_quality        = 1000,
};

struct apq8064_timer_softc *apq8064_timer_sc = NULL;

static struct resource_spec apq8064_timer_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static int
apq8064_timer_probe(device_t dev)
{
	struct apq8064_timer_softc *sc;

	sc = device_get_softc(dev);

	if (!ofw_bus_is_compatible(dev, "qcom,msm-timer"))
		return (ENXIO);

	device_set_desc(dev, "Qualcomm APQ8064 timer");
	return (BUS_PROBE_DEFAULT);
}

static int
apq8064_timer_attach(device_t dev)
{
	struct apq8064_timer_softc *sc;
	int err;

	sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, apq8064_timer_spec, sc->res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->sc_dev = dev;
	sc->sc_bst = rman_get_bustag(sc->res[0]);
	sc->sc_bsh = rman_get_bushandle(sc->res[0]);

	/* Setup and enable the timer interrupt */
	err = bus_setup_intr(dev, sc->res[1], INTR_TYPE_CLK, apq8064_timer_hardclock,
	    NULL, sc, &sc->sc_ih);
	if (err != 0) {
		bus_release_resources(dev, apq8064_timer_spec, sc->res);
		device_printf(dev, "Unable to setup the clock irq handler, "
		    "err = %d\n", err);
		return (ENXIO);
	}

	/* Enable timers */
	timer_write_4(sc, GPT_ENABLE, GPT_ENABLE_EN);
	timer_write_4(sc, DGT_ENABLE, DGT_ENABLE_EN);

//	timer_write_4(sc, GPT1_CLEAR, 0);
//	timer_write_4(sc, GPT1_ENABLE, GPT_ENABLE_EN);

	sc->timer0_freq = SYS_TIMER_CLKSRC;

	/* Set desired frequency in event timer and timecounter */
	sc->et.et_frequency = sc->timer0_freq;
	sc->et.et_name = "apq8064_timer Eventtimer";
	sc->et.et_flags = ET_FLAGS_ONESHOT | ET_FLAGS_PERIODIC;
	sc->et.et_quality = 1000;
	sc->et.et_min_period = (0x00000005LLU << 32) / sc->et.et_frequency;
	sc->et.et_max_period = (0xfffffffeLLU << 32) / sc->et.et_frequency;
	sc->et.et_start = apq8064_timer_timer_start;
	sc->et.et_stop = apq8064_timer_timer_stop;
	sc->et.et_priv = sc;
	et_register(&sc->et);

	if (device_get_unit(dev) == 0)
		apq8064_timer_sc = sc;

	apq8064_timer_timecounter.tc_frequency = sc->timer0_freq;
	tc_init(&apq8064_timer_timecounter);

	if (bootverbose) {
		device_printf(sc->sc_dev, "clock: hz=%d stathz = %d\n", hz, stathz);

		device_printf(sc->sc_dev, "event timer clock frequency %u\n", 
		    sc->timer0_freq);
		device_printf(sc->sc_dev, "timecounter clock frequency %lld\n", 
		    apq8064_timer_timecounter.tc_frequency);
	}

	apq8064_timer_initialized = 1;

	return (0);
}

static int
apq8064_timer_timer_start(struct eventtimer *et, sbintime_t first,
    sbintime_t period)
{
	struct apq8064_timer_softc *sc;
	uint32_t count;
	uint32_t val;

	sc = (struct apq8064_timer_softc *)et->et_priv;

	if (period != 0)
		sc->sc_period = ((uint32_t)et->et_frequency * period) >> 32;
	else
		sc->sc_period = 0;
	if (first != 0)
		count = ((uint32_t)et->et_frequency * first) >> 32;
	else
		count = sc->sc_period;

	/* Update timer values */
	timer_write_4(sc, DGT_MATCH_VAL, count);

	timer_write_4(sc, DGT_CLEAR, 0);

	val = timer_read_4(sc, DGT_ENABLE);
	if (period != 0) {
		/* periodic */
		val |= DGT_ENABLE_CLR_ON_MATCH_EN;
	} else {
		/* oneshot */
		val &= ~DGT_ENABLE_CLR_ON_MATCH_EN;
	}
	/* Enable timer0 */
	val |= DGT_ENABLE_EN;
	timer_write_4(sc, DGT_ENABLE, val);

	return (0);
}

static int
apq8064_timer_timer_stop(struct eventtimer *et)
{
	struct apq8064_timer_softc *sc;
	uint32_t val;

	sc = (struct apq8064_timer_softc *)et->et_priv;

	/* Disable timer0 */
	val = timer_read_4(sc, DGT_ENABLE);
	val &= ~DGT_ENABLE_EN;
	timer_write_4(sc, DGT_ENABLE, val);
	while (timer_read_4(sc, SPSS_TIMER_STATUS) & SPSS_TIMER_STATUS_DGT_EN)
		;
        timer_write_4(sc, DGT_CLEAR, 0);
	while (timer_read_4(sc, SPSS_TIMER_STATUS) & SPSS_TIMER_STATUS_DGT_EN)
		;
	sc->sc_period = 0;

	return (0);
}

int
apq8064_timer_get_timerfreq(struct apq8064_timer_softc *sc)
{

	return (sc->timer0_freq);
}

void
cpu_initclocks(void)
{

	cpu_initclocks_bsp();
}

static int
apq8064_timer_hardclock(void *arg)
{
	struct apq8064_timer_softc *sc;
	uint32_t val;

	sc = (struct apq8064_timer_softc *)arg;

//	timer_write_4(sc, DGT_CLEAR, 0);

	val = timer_read_4(sc, DGT_ENABLE);
	/*
	 * Disabled autoreload and sc_period > 0 means 
	 * timer_start was called with non NULL first value.
	 * Now we will set periodic timer with the given period 
	 * value.
	 */
	if ((val & (1<<1)) == 0 && sc->sc_period > 0) {
		/* Update timer */
		timer_write_4(sc, DGT_MATCH_VAL, sc->sc_period);

		/* Make periodic and enable */
		val |= DGT_ENABLE_CLR_ON_MATCH_EN | DGT_ENABLE_EN;
		timer_write_4(sc, DGT_ENABLE, val);
	}

	if (sc->et.et_active)
		sc->et.et_event_cb(&sc->et, sc->et.et_arg);

	return (FILTER_HANDLED);
}

u_int
apq8064_timer_get_timecount(struct timecounter *tc)
{
	u_int timecount;

	if (apq8064_timer_sc == NULL)
		return (0);

	timecount = timer_read_4(apq8064_timer_sc, GPT_COUNT_VAL);

	return (timecount);
}

static device_method_t apq8064_timer_methods[] = {
	DEVMETHOD(device_probe,		apq8064_timer_probe),
	DEVMETHOD(device_attach,	apq8064_timer_attach),

	DEVMETHOD_END
};

static driver_t apq8064_timer_driver = {
	"apq8064_timer",
	apq8064_timer_methods,
	sizeof(struct apq8064_timer_softc),
};

static devclass_t apq8064_timer_devclass;

DRIVER_MODULE(apq8064_timer, simplebus, apq8064_timer_driver, apq8064_timer_devclass, 0, 0);

void
DELAY(int usec)
{
	uint32_t counter;
	uint64_t end, now;

	if (!apq8064_timer_initialized) {
		for (; usec > 0; usec--)
			for (counter = 50; counter > 0; counter--)
				cpufunc_nullop();
		return;
	}
/*
	timer_write_4(apq8064_timer_sc, GPT_CLEAR, 0);
	timer_write_4(apq8064_timer_sc, GPT_ENABLE, 0);
	while (timer_read_4(apq8064_timer_sc, GPT_COUNT_VAL) != 0)
		;

	timer_write_4(apq8064_timer_sc, GPT_ENABLE, GPT_ENABLE_EN);
*/
	now = timer_read_4(apq8064_timer_sc, GPT_COUNT_VAL);
	end = now + (apq8064_timer_sc->timer0_freq / 1000000) * (usec + 1);

	while (now < end)
		now = timer_read_4(apq8064_timer_sc, GPT_COUNT_VAL);

//	timer_write_4(apq8064_timer_sc, GPT_ENABLE, 0);
//	timer_write_4(apq8064_timer_sc, GPT_CLEAR, 0);
}

