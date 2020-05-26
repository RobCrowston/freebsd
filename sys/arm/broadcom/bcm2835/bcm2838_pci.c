/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2020 Dr Robert Harvey Crowston <crowston@protonmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *
 * $FreeBSD$
 *
 */

/*
 * BCM2838-compatible PCI-express controller.
 *
 * Broadcom likes to give the same chip lots of different names. The name of
 * this driver is taken from the Raspberry Pi 4 Broadcom 2838 chip.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/intr.h>
#include <sys/mutex.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/pci/pci_host_generic.h>
#include <dev/pci/pci_host_generic_fdt.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include "pcib_if.h"
#include "msi_if.h"

extern struct bus_space memmap_bus;

#define	PCIE_BUS_SHIFT		20
#define	PCIE_SLOT_SHIFT		15
#define	PCIE_FUNC_SHIFT		12
#define	PCIE_SLOT_MASK		0x1F
#define	PCIE_FUNC_MASK		0x07

#define REG_CONTROLLER_HW_REV			0x406c

#define REG_BRIDGE_CTRL				0x9210
#define BRIDGE_DISABLE_FLAG	0x1
#define BRIDGE_RESET_FLAG	0x2
#define REG_BRIDGE_SERDES_MODE			0x4204
#define REG_BRIDGE_CONFIG			0x4008
#define REG_BRIDGE_MEM_WINDOW_LO		0x4034
#define REG_BRIDGE_MEM_WINDOW_HI		0x4038
#define REG_BRIDGE_MEM_WINDOW_1			0x403c
#define REG_BRIDGE_GISB_WINDOW			0x402c
#define REG_BRIDGE_STATE			0x4068
#define REG_BRIDGE_LINK_STATE			0x00bc
#define REG_BRIDGE_BUS_WINDOW_LOW		0x400c
#define REG_BRIDGE_BUS_WINDOW_HIGH		0x4010
#define REG_BRIDGE_CPU_WINDOW_LOW		0x4070
#define REG_BRIDGE_CPU_WINDOW_START_HIGH	0x4080
#define REG_BRIDGE_CPU_WINDOW_END_HIGH		0x4084

#define REG_MSI_ADDR_LOW			0x4044
#define REG_MSI_ADDR_HIGH			0x4048
#define REG_MSI_CONFIG_MAGIC			0x404c
#define REG_MSI_CLR				0x4508
#define REG_MSI_MASK_CLR			0x4514
#define REG_MSI_RAISED				0x4500
#define REG_MSI_EOI				0x4060
#define MAX_MSI			32

#define REG_EP_CONFIG_CHOICE			0x9000
#define REG_EP_CONFIG_DATA			0x8000

struct bcm_pcib_irqsrc {
	struct intr_irqsrc	isrc;
	u_int			irq;
	bool			allocated;
};

/* In generic_pcie_fdt_route_interrupt() our softc is cast to a
 * generic_pcie_fdt_softc to access pci_iinfo, so keep the first two members
 * below in order until this is refactored. */
struct bcm_pcib_softc {
	struct generic_pcie_core_softc	base;
	struct ofw_bus_iinfo		pci_iinfo;
	device_t			dev;
	struct mtx			msi_mtx;
	struct resource 		*msi_irq_res;
	void				*msi_intr_cookie;
	struct bcm_pcib_irqsrc		*msi_isrcs;
	bus_addr_t			msi_addr;
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm2711-pcie",			1},
	{"brcm,bcm7211-pcie",			1},
	{"brcm,bcm7445-pcie",			1},

	{NULL,					0}
};

static int
bcm_pcib_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev,
	    "BCM2838-compatible PCI-express controller");
	return (BUS_PROBE_DEFAULT);
}

static void
bcm_pcib_set_reg(struct bcm_pcib_softc *sc, uint32_t reg, uint32_t val)
{

	bus_space_write_4(sc->base.bst, sc->base.bsh, reg, htole32(val));
}

static uint32_t
bcm_pcib_read_reg(struct bcm_pcib_softc *sc, uint32_t reg)
{

	return (le32toh(bus_space_read_4(sc->base.bst, sc->base.bsh, reg)));
}

static uint32_t
encode_cpu_window_low(bus_addr_t phys_base, bus_size_t size)
{

	return (((phys_base >> 0x10) & 0xfff0) |
	    ((phys_base + size - 1) & 0xfff00000));
}

static uint32_t
encode_cpu_window_start_high(bus_addr_t phys_base)
{

	return ((phys_base >> 0x20) & 0xff);
}

static uint32_t
encode_cpu_window_end_high(bus_addr_t phys_base, bus_size_t size)
{

	return (((phys_base + size - 1) >> 0x20) & 0xff);
}

static void
bcm_pcib_reset_controller(struct bcm_pcib_softc *sc)
{
	uint32_t val;

	val = bcm_pcib_read_reg(sc, REG_BRIDGE_CTRL);
	val = val | BRIDGE_RESET_FLAG | BRIDGE_DISABLE_FLAG;
	bcm_pcib_set_reg(sc, REG_BRIDGE_CTRL, val);

	DELAY(100);

	val = bcm_pcib_read_reg(sc, REG_BRIDGE_CTRL);
	val = val & ~BRIDGE_RESET_FLAG;
	bcm_pcib_set_reg(sc, REG_BRIDGE_CTRL, val);

	DELAY(100);

	bcm_pcib_set_reg(sc, REG_BRIDGE_SERDES_MODE, 0);

	DELAY(100);
}

static void
bcm_pcib_enable_controller(struct bcm_pcib_softc *sc)
{
	uint32_t val;

	val = bcm_pcib_read_reg(sc, REG_BRIDGE_CTRL);
	val = val & ~BRIDGE_DISABLE_FLAG;
	bcm_pcib_set_reg(sc, REG_BRIDGE_CTRL, val);

	DELAY(100);
}

static int
bcm_pcib_check_window(device_t dev)
{
	struct bcm_pcib_softc *sc;
	int error = 0, i;

	sc = device_get_softc(dev);

	/* First range needs to be non-zero. */
	if (sc->base.ranges[0].size == 0) {
		device_printf(dev, "error: first outbound memory range "
		    "(pci addr: 0x%jx, cpu addr: 0x%jx) has zero size.\n",
		    sc->base.ranges[0].pci_base, sc->base.ranges[0].phys_base);
		error = ENXIO;
	}

	/* Only first range matters. */
	for (i = 1; (bootverbose || error) && i < MAX_RANGES_TUPLES; ++i) {
		if (sc->base.ranges[i].size > 0)
			device_printf(dev,
			    "note: outbound memory range %d (pci addr: 0x%jx, "
			    "cpu addr: 0x%jx, size: 0x%jx) will be ignored.\n",
			    i, sc->base.ranges[i].pci_base,
			    sc->base.ranges[i].phys_base,
			    sc->base.ranges[i].size);
	}

	return (error);
}

static const char *
bcm_pcib_link_state_string(uint32_t mode)
{

	switch(mode & PCIEM_LINK_STA_SPEED) {
	case 0:
		return ("not up");
	case 1:
		return ("2.5 GT/s");
	case 2:
		return ("5.0 GT/s");
	case 4:
		return ("8.0 GT/s");
	default:
		return ("unknown");
	}
}

static struct resource *
bcm_pcib_alloc_resource(device_t dev, device_t child, int type,
    int *rid, rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct bcm_pcib_softc *sc;
	struct resource *res;
	struct rman *rm;

	sc = device_get_softc(dev);

#if defined(NEW_PCIB) && defined(PCI_RES_BUS)
	if (type == PCI_RES_BUS) {
		return (pci_domain_alloc_bus(sc->base.ecam, child, rid, start,
		    end, count, flags));
	}
#endif
	rm = generic_pcie_rman(&sc->base, type);
	if (rm == NULL)
		return (BUS_ALLOC_RESOURCE(device_get_parent(dev), child, type,
		    rid, start, end, count, flags));

	/* Adjust address to a PCI bus address. These addresses are provided
	 * to us as zero offset, not physical offset. */
	start += sc->base.ranges[0].pci_base;
	end += sc->base.ranges[0].pci_base;

	if (bootverbose) {
		device_printf(dev,
		    "rman_reserve_resource: start=%#jx, end=%#jx, count=%#jx\n",
		    start, end, count);
	}

	res = rman_reserve_resource(rm, start, end, count, flags, child);
	if (res == NULL)
		goto fail;

	rman_set_rid(res, *rid);

	if (flags & RF_ACTIVE)
		if (bus_activate_resource(child, type, *rid, res)) {
			rman_release_resource(res);
			goto fail;
		}

	return (res);

fail:
	device_printf(dev, "%s FAIL: type=%d, rid=%d, "
	    "start=%016jx, end=%016jx, count=%016jx, flags=%x\n",
	    __func__, type, *rid, start, end, count, flags);

	return (NULL);
}

static bus_addr_t
pci_addr_to_phys_addr(struct pcie_range *ranges, bus_addr_t pci_addr)
{
	bus_addr_t pci_start, pci_end, phys_start;

	pci_start  = ranges[0].pci_base;
	pci_end    = ranges[0].pci_base + ranges[0].size;
	phys_start = ranges[0].phys_base;

	if (pci_addr >= pci_start && pci_addr < pci_end)
		return ((pci_addr - pci_start) + phys_start);

	/* Illegal address. */
	return (0);
}

static int
bcm_pcib_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *r)
{
	int err;
	struct bcm_pcib_softc *sc;
	bus_addr_t paddr;
	bus_size_t psize;
	bus_space_handle_t vaddr;

	if ((err = rman_activate_resource(r)) != 0)
		return (err);

	/*
	 * If this is a memory resource, map it into the kernel.
	 */
	if (type == SYS_RES_MEMORY || type == SYS_RES_IOPORT) {
		sc = device_get_softc(bus);

		paddr = (bus_addr_t)rman_get_start(r);
		psize = (bus_size_t)rman_get_size(r);

		/* Adjust the bus address to a CPU address. */
		paddr = pci_addr_to_phys_addr(sc->base.ranges, paddr);

		err = bus_space_map(&memmap_bus, paddr, psize, 0, &vaddr);
		if (err != 0) {
			rman_deactivate_resource(r);
			return (err);
		}
		rman_set_bustag(r, &memmap_bus);
		rman_set_virtual(r, (void *)vaddr);
		rman_set_bushandle(r, vaddr);
	} else if (type == SYS_RES_IRQ) {
		err = intr_activate_irq(child, r);
		if (err != 0) {
			rman_deactivate_resource(r);
			return (err);
		}
	}

	return (0);
}

static bus_size_t
bcm_get_offset_and_prepare_config(struct bcm_pcib_softc *sc, u_int bus,
    u_int slot, u_int func, u_int reg)
{
	/* Config for a device is only available through a narrow window for
	 * one device at a time. */
	uint32_t func_index;

	if (bus == 0 && slot == 0 && func == 0)
		/* Special case for root device; its config is always available
		 * through the zero-offset. */
		return (reg);

	/* Tell the controller to show us the config in question. */
	func_index = ((bus << PCIE_BUS_SHIFT)) |
	    ((slot & PCIE_SLOT_MASK) << PCIE_SLOT_SHIFT) |
	    ((func & PCIE_FUNC_MASK) << PCIE_FUNC_SHIFT);
	bcm_pcib_set_reg(sc, REG_EP_CONFIG_CHOICE, func_index);

	return (REG_EP_CONFIG_DATA + reg);
}

static bool
bcm_pcib_is_valid_quad(struct bcm_pcib_softc *sc, u_int bus, u_int slot,
    u_int func, u_int reg)
{

	if ((bus < sc->base.bus_start) || (bus > sc->base.bus_end))
		return (false);
	if ((slot > PCI_SLOTMAX) || (func > PCI_FUNCMAX) || (reg > PCIE_REGMAX))
		return (false);

	if (bus == 0 && slot == 0 && func == 0)
		return (true);
	if (bus == 0)
		/* Probing other slots and funcs on bus 0 will lock up the
		 * memory controller. */
	    return (false);

	return (true);
}

static uint32_t
bcm_pcib_read_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, int bytes)
{
	struct bcm_pcib_softc *sc;
	bus_space_handle_t h;
	bus_space_tag_t	t;
	uint32_t offset, data;

	sc = device_get_softc(dev);
	if (!bcm_pcib_is_valid_quad(sc, bus, slot, func, reg))
	    return (~0U);

	offset = bcm_get_offset_and_prepare_config(sc, bus, slot, func, reg);

	t = sc->base.bst;
	h = sc->base.bsh;

	switch (bytes) {
	case 1:
		data = bus_space_read_1(t, h, offset);
		break;
	case 2:
		data = le16toh(bus_space_read_2(t, h, offset));
		break;
	case 4:
		data = le32toh(bus_space_read_4(t, h, offset));
		break;
	default:
		return (~0U);
	}

	return (data);
}

static void
bcm_pcib_write_config(device_t dev, u_int bus, u_int slot,
    u_int func, u_int reg, uint32_t val, int bytes)
{
	struct bcm_pcib_softc *sc;
	bus_space_handle_t h;
	bus_space_tag_t	t;
	uint32_t offset;

	sc = device_get_softc(dev);
	if (!bcm_pcib_is_valid_quad(sc, bus, slot, func, reg))
	    return;

	offset = bcm_get_offset_and_prepare_config(sc, bus, slot, func, reg);

	t = sc->base.bst;
	h = sc->base.bsh;

	switch (bytes) {
	case 1:
		bus_space_write_1(t, h, offset, val);
		break;
	case 2:
		bus_space_write_2(t, h, offset, htole16(val));
		break;
	case 4:
		bus_space_write_4(t, h, offset, htole32(val));
		break;
	default:
		return;
	}
}

static void
bcm_pcib_msi_intr_process(struct bcm_pcib_softc *sc, uint32_t interrupt_bitmap,
    struct trapframe *tf)
{
	struct bcm_pcib_irqsrc *irqsrc;
	uint32_t bit, irq;

	while ((bit = ffs(interrupt_bitmap))) {
		irq = bit - 1;

		/* Acknowledge interrupt. */
		bcm_pcib_set_reg(sc, REG_MSI_CLR, 1 << irq);

		/* Send EOI. */
		bcm_pcib_set_reg(sc, REG_MSI_EOI, 1);

		/* Despatch to handler. */
		irqsrc = &sc->msi_isrcs[irq];
		if (intr_isrc_dispatch(&irqsrc->isrc, tf))
			device_printf(sc->dev,
			    "note: unexpected interrupt triggered.\n");

		/* Done with this interrupt. (We'll refresh from the controller
		 * in the outer loop after the first pass.) */
		interrupt_bitmap &= ~(1 << irq);
	}
}

static int
bcm_pcib_msi_intr(void *arg)
{
	struct bcm_pcib_softc *sc;
	struct trapframe *tf;
	uint32_t interrupt_bitmap;

	sc = (struct bcm_pcib_softc *)arg;
	tf = curthread->td_intr_frame;

	while ((interrupt_bitmap = bcm_pcib_read_reg(sc, REG_MSI_RAISED)))
		bcm_pcib_msi_intr_process(sc, interrupt_bitmap, tf);

	return (FILTER_HANDLED);
}

static int
bcm_pcib_alloc_msi(device_t dev, device_t child, int count, int maxcount,
    device_t *pic, struct intr_irqsrc **srcs)
{
	struct bcm_pcib_softc *sc;
	int first_int, i;

	sc = device_get_softc(dev);
	mtx_lock(&sc->msi_mtx);

	/* Find a continguous region of free message-signalled interrupts. */
	for (first_int = 0; first_int + count < MAX_MSI; ) {
		for (i = first_int; i < first_int + count; ++i) {
			if (sc->msi_isrcs[i].allocated)
				goto next;
		}
		goto found;
next:
		first_int = i + 1;
	}

	/* No appropriate region available. */
	mtx_unlock(&sc->msi_mtx);
	device_printf(dev, "warning: failed to allocate %d MSI messages.\n",
	    count);
	return (ENXIO);

found:
	/* Mark the messages as in use. */
	for (i = 0; i < count; ++i) {
		sc->msi_isrcs[i + first_int].allocated = true;
		srcs[i] = &(sc->msi_isrcs[i + first_int].isrc);
	}

	mtx_unlock(&sc->msi_mtx);
	*pic = device_get_parent(dev);

	return (0);
}

static int
bcm_pcib_map_msi(device_t dev, device_t child, struct intr_irqsrc *isrc,
    uint64_t *addr, uint32_t *data)
{
	struct bcm_pcib_softc *sc;
	struct bcm_pcib_irqsrc *msi_msg;

	sc = device_get_softc(dev);
	msi_msg = (struct bcm_pcib_irqsrc *) isrc;

	*addr = sc->msi_addr;
	*data = 0x6540 | msi_msg->irq;
	return (0);
}

static int
bcm_pcib_release_msi(device_t dev, device_t child, int count,
    struct intr_irqsrc **isrc)
{
	struct bcm_pcib_softc *sc;
	struct bcm_pcib_irqsrc *msi_isrc;
	int i;

	sc = device_get_softc(dev);
	mtx_lock(&sc->msi_mtx);

	for (i = 0; i < count; i++) {
		msi_isrc = (struct bcm_pcib_irqsrc *) isrc[i];
		msi_isrc->allocated = false;
	}

	mtx_unlock(&sc->msi_mtx);
	return (0);
}

static int
bcm_pcib_msi_attach(device_t dev)
{
	struct bcm_pcib_softc *sc;
	phandle_t node, xref;
	char const *bcm_name;
	int i, rid;

	sc = device_get_softc(dev);
	sc->msi_addr = 0xffffffffc;

	/* Clear any pending interrupts. */
	bcm_pcib_set_reg(sc, REG_MSI_CLR, 0xffffffff);

	rid = 1;
	sc->msi_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->msi_irq_res == NULL) {
		device_printf(dev, "could not allocate MSI irq resource.\n");
		return (ENXIO);
	}

	sc->msi_isrcs = malloc(sizeof(*sc->msi_isrcs) * MAX_MSI, M_DEVBUF,
	    M_WAITOK | M_ZERO);

	int error = bus_setup_intr(dev, sc->msi_irq_res, INTR_TYPE_BIO |
	    INTR_MPSAFE, bcm_pcib_msi_intr, NULL, sc, &sc->msi_intr_cookie);
	if (error) {
		device_printf(dev, "error: failed to setup MSI handler.\n");
		return (ENXIO);
	}

	bcm_name = device_get_nameunit(dev);
	for (i = 0; i < MAX_MSI; i++) {
		sc->msi_isrcs[i].irq = i;
		error = intr_isrc_register(&sc->msi_isrcs[i].isrc, dev, 0,
		    "%s,%u", bcm_name, i);
		if (error) {
			device_printf(dev,
			"error: failed to register interrupt %d.\n", i);
			return (ENXIO);
		}
	}

	node = ofw_bus_get_node(dev);
	xref = OF_xref_from_node(node);
	OF_device_register_xref(xref, dev);

	if (intr_msi_register(dev, xref) != 0)
		return (ENXIO);

	mtx_init(&sc->msi_mtx, "bcm_pcib: msi_mtx", NULL, MTX_DEF);

	bcm_pcib_set_reg(sc, REG_MSI_MASK_CLR, 0xffffffff);
	bcm_pcib_set_reg(sc, REG_MSI_ADDR_LOW,  (sc->msi_addr & 0xffffffff) |
	    0x1);
	bcm_pcib_set_reg(sc, REG_MSI_ADDR_HIGH, (sc->msi_addr >> 32));
	bcm_pcib_set_reg(sc, REG_MSI_CONFIG_MAGIC, 0xffe06540);

	return (0);
}

static int
bcm_pcib_attach(device_t dev)
{
	struct bcm_pcib_softc *sc;
	bus_addr_t phys_base, pci_base;
	bus_size_t size;
	uint32_t hardware_rev, bridge_state, link_state;
	phandle_t node;
	int error, tries;

	sc = device_get_softc(dev);
	sc->dev = dev;

	error = pci_host_generic_setup_fdt(dev, &sc->base);
	if (error)
		return (error);

	error = bcm_pcib_check_window(dev);
	if (error)
		return (error);

	bcm_pcib_reset_controller(sc);

	hardware_rev = bcm_pcib_read_reg(sc, REG_CONTROLLER_HW_REV) & 0xffff;
	device_printf(dev, "hardware identifies as revision 0x%x.\n",
	    hardware_rev);

	/* Set PCI->CPU memory window. This encodes the inbound window showing
	 * up to 4 GiB of system memory to the controller, with zero offset.
	 * Thus, from the perspective of a device on the PCI-E bus, there is a
	 * 1:1 map from PCI-E bus addresses to system memory addresses. */
	bcm_pcib_set_reg(sc, REG_BRIDGE_MEM_WINDOW_LO, 0x11);
	bcm_pcib_set_reg(sc, REG_BRIDGE_MEM_WINDOW_HI, 0);
	bcm_pcib_set_reg(sc, REG_BRIDGE_GISB_WINDOW, 0);
	bcm_pcib_set_reg(sc, REG_BRIDGE_MEM_WINDOW_1, 0);
	bcm_pcib_set_reg(sc, REG_BRIDGE_CONFIG, 0x88003000);

	bcm_pcib_enable_controller(sc);

	/* Wait for controller to start. */
	for(tries = 0; ; ++tries) {
		bridge_state = bcm_pcib_read_reg(sc, REG_BRIDGE_STATE);

		if ((bridge_state & 0x30) == 0x30)
			/* Controller ready. */
			break;

		if (tries > 100) {
			device_printf(dev,
			    "error: controller failed to start.\n");
			return (ENXIO);
		}

		DELAY(1000);
	}

	link_state = bcm_pcib_read_reg(sc, REG_BRIDGE_LINK_STATE) >> 0x10;
	if (!link_state) {
		device_printf(dev, "error: controller started but link is not "
		    "up.\n");
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(dev, "note: reported link speed is %s.\n",
		    bcm_pcib_link_state_string(link_state));

	/* Set CPU->PCI memory window. The map in this direction is not 1:1.
	 * Addresses seen by the CPU need to be adjusted to make sense to the
	 * controller as they pass through the window. */
	pci_base  = sc->base.ranges[0].pci_base;
	phys_base = sc->base.ranges[0].phys_base;
	size      = sc->base.ranges[0].size;

	bcm_pcib_set_reg(sc, REG_BRIDGE_BUS_WINDOW_LOW, pci_base & 0xffffffff);
	bcm_pcib_set_reg(sc, REG_BRIDGE_BUS_WINDOW_HIGH, pci_base >> 32);

	bcm_pcib_set_reg(sc, REG_BRIDGE_CPU_WINDOW_LOW,
	    encode_cpu_window_low(phys_base, size));
	bcm_pcib_set_reg(sc, REG_BRIDGE_CPU_WINDOW_START_HIGH,
	    encode_cpu_window_start_high(phys_base));
	bcm_pcib_set_reg(sc, REG_BRIDGE_CPU_WINDOW_END_HIGH,
	    encode_cpu_window_end_high(phys_base, size));

	/* The controller starts up thinking it is an endpoint; reconfigure it
	 * as a bridge. */
	bcm_pcib_set_reg(sc, 0x043c, 0x060400);

	bcm_pcib_set_reg(sc, REG_BRIDGE_SERDES_MODE, 0x2);
	DELAY(100);

	/* Configure interrupts. */
	node = ofw_bus_get_node(dev);
	ofw_bus_setup_iinfo(node, &sc->pci_iinfo, sizeof(cell_t));
	error = bcm_pcib_msi_attach(dev);
	if (error)
		return (error);

	/* Done. */
	device_add_child(dev, "pci", -1);
	return (bus_generic_attach(dev));
}

/*
 * Device method table.
 */
static device_method_t bcm_pcib_methods[] = {
	/* Device interface. */
	DEVMETHOD(device_probe,			bcm_pcib_probe),
	DEVMETHOD(device_attach,		bcm_pcib_attach),

	/* Bus interface. */
	DEVMETHOD(bus_alloc_resource,		bcm_pcib_alloc_resource),
	DEVMETHOD(bus_activate_resource,	bcm_pcib_activate_resource),

	/* PCIB interface. */
	DEVMETHOD(pcib_read_config,		bcm_pcib_read_config),
	DEVMETHOD(pcib_write_config,		bcm_pcib_write_config),

	/* MSI interface. */
	DEVMETHOD(msi_alloc_msi,		bcm_pcib_alloc_msi),
	DEVMETHOD(msi_release_msi,		bcm_pcib_release_msi),
	DEVMETHOD(msi_map_msi,			bcm_pcib_map_msi),

	DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, bcm_pcib_driver, bcm_pcib_methods,
    sizeof(struct bcm_pcib_softc), generic_pcie_fdt_driver);

static devclass_t bcm_pcib_devclass;
DRIVER_MODULE(pcib, simplebus, bcm_pcib_driver, bcm_pcib_devclass, 0, 0);

