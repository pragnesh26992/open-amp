/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2017 Xilinx, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**************************************************************************
 * FILE NAME
 *
 *       platform_info.c
 *
 * DESCRIPTION
 *
 *       This file define Xilinx ZynqMP R5 to A53 platform specific 
 *       remoteproc implementation.
 *
 **************************************************************************/
#include <metal/alloc.h>
#include <metal/atomic.h>
#include <metal/io.h>
#include <metal/irq.h>
#include <metal/device.h>
#include <metal/utilities.h>
#include <openamp/remoteproc.h>
#include <openamp/rpmsg_virtio.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/un.h>
#include "platform_info.h"

#define ISR(x)        (0x2200 + ((x) << 2))
#define PCSS_SCR_BX2_MCCI_RD_IND	0x874

atomic_int ipc_nokick;

#ifndef RPMSG_NO_IPI
static int sifive_u_linux_bx2_proc_irq_handler(int vect_id, void *data)
{
        struct remoteproc *rproc = data;
        struct remoteproc_priv *prproc;
        unsigned int ipi_intr_status;

        (void)vect_id;
        if (!rproc)
                return METAL_IRQ_NOT_HANDLED;
        prproc = rproc->priv;
        ipi_intr_status = (unsigned int)metal_io_read32(prproc->ipi_io,
                                                        ISR(0));

//	metal_io_write32(prproc->ipi_io, BX2_DMSS_GPOUT_ADDR, 0x0);
	atomic_flag_clear(&prproc->ipi_nokick);
	return METAL_IRQ_HANDLED;
}

static int sifive_u_linux_bx2_ipc_ack_handler(int vect_id, void *data)
{
        struct remoteproc *rproc = data;
        struct remoteproc_priv *prproc;
        unsigned int ipi_intr_status;

        (void)vect_id;
        if (!rproc)
                return METAL_IRQ_NOT_HANDLED;
        prproc = rproc->priv;
        ipi_intr_status = (unsigned int)metal_io_read32(prproc->ipi_io,
                                                        ISR(0));

        ipi_intr_status = (unsigned int)metal_io_read32(prproc->ipi_ack_io,
                                                        PCSS_SCR_BX2_MCCI_RD_IND);
	metal_io_write32(prproc->ipi_ack_io, PCSS_SCR_BX2_MCCI_RD_IND, 0xF);
	atomic_flag_clear(&ipc_nokick);
	return METAL_IRQ_HANDLED;
}
#endif /* !RPMSG_NO_IPI */

static struct remoteproc *
sifive_u_linux_proc_init(struct remoteproc *rproc,
			 struct remoteproc_ops *ops, void *arg)
{
	struct remoteproc_priv *prproc = arg;
	struct metal_device *dev;
#ifndef RPMSG_NO_IPI
	unsigned int irq_vect;
#endif /* !RPMSG_NO_IPI */
	metal_phys_addr_t mem_pa;
	int ret;

	if (!rproc || !prproc || !ops)
		return NULL;
	rproc->priv = prproc;
	rproc->ops = ops;
	prproc->ipi_dev = NULL;
	prproc->shm_dev = NULL;
	/* Get shared memory device */
	ret = metal_device_open(prproc->shm_bus_name, prproc->shm_name,
			&dev);
	if (ret) {
		fprintf(stderr, "ERROR: failed to open shm device: %d.\r\n", ret);
		goto err1;
	}
	printf("Successfully open shm device.\r\n");
	prproc->shm_dev = dev;
	prproc->shm_io = metal_device_io_region(dev, 0);
	if (!prproc->shm_io)
		goto err2;

#ifdef RPMSG_NO_IPI
	/* Get poll shared memory device */
	ret = metal_device_open(prproc->shm_poll_bus_name,
				prproc->shm_poll_name,
				&dev);
	if (ret) {
		fprintf(stderr,
			"ERROR: failed to open shm poll device: %d.\r\n",
			ret);
		goto err1;
	}
	printf("Successfully open shm poll device.\r\n");
	prproc->shm_poll_dev = dev;
	prproc->shm_poll_io = metal_device_io_region(dev, 0);
	if (!prproc->shm_poll_io)
		goto err2;
	metal_io_write32(prproc->shm_poll_io, 0, !POLL_STOP);
#endif /* RPMSG_NO_IPI */

	mem_pa = metal_io_phys(prproc->shm_io, 0);
	remoteproc_init_mem(&prproc->shm_mem, "shm", mem_pa, mem_pa,
			    metal_io_region_size(prproc->shm_io),
			    prproc->shm_io);
	remoteproc_add_mem(rproc, &prproc->shm_mem);
	printf("Successfully added shared memory\r\n");
	/* Get IPI device */
#ifndef RPMSG_NO_IPI
	ret = metal_device_open(prproc->ipi_bus_name, prproc->ipi_name,
				&dev);
	if (ret) {
		printf("failed to open ipi device: %d.\r\n", ret);
		goto err2;
	}
	prproc->ipi_dev = dev;
	prproc->ipi_io = metal_device_io_region(dev, 0);
	if (!prproc->ipi_io)
		goto err3;
	printf("Successfully probed IPI device\r\n");
	atomic_store(&prproc->ipi_nokick, 1);
	atomic_store(&ipc_nokick, 1);

	/* Register interrupt handler and enable interrupt */
	irq_vect = (uintptr_t)dev->irq_info;
	metal_irq_register(irq_vect, sifive_u_linux_bx2_proc_irq_handler, rproc);
	metal_irq_enable(irq_vect);

	ret = metal_device_open(prproc->ipi_ack_bus_name, prproc->ipi_ack_name,
				&dev);
	if (ret) {
		printf("failed to open ipi device: %d.\r\n", ret);
		goto err2;
	}
	prproc->ipi_ack_dev = dev;
	prproc->ipi_ack_io = metal_device_io_region(dev, 0);
	if (!prproc->ipi_ack_io)
		goto err4;
	printf("Successfully probed IPI ack device\r\n");

#if 0
	/* Register interrupt handler and enable interrupt */
	irq_vect = (uintptr_t)dev->irq_info;
	metal_irq_register(irq_vect, sifive_u_linux_bx2_ipc_ack_handler, rproc);
	metal_irq_enable(irq_vect);
#endif
	printf("Successfully initialized Linux r5 remoteproc.\r\n");
	return rproc;
#endif /* !RPMSG_NO_IPI */
	printf("Successfully initialized Linux r5 remoteproc.\r\n");
	return rproc;
#ifndef RPMSG_NO_IPI
err4:
	metal_device_close(prproc->ipi_ack_dev);
err3:
	metal_device_close(prproc->ipi_dev);
#endif /* !RPMSG_NO_IPI */
err2:
	metal_device_close(prproc->shm_dev);
err1:
	return NULL;
}

static void sifive_u_linux_proc_remove(struct remoteproc *rproc)
{
	struct remoteproc_priv *prproc;
#ifndef RPMSG_NO_IPI
	struct metal_device *dev;
#endif /* !RPMSG_NO_IPI */

	if (!rproc)
		return;
	prproc = rproc->priv;
#ifndef RPMSG_NO_IPI
	dev = prproc->ipi_dev;
	if (dev) {
		metal_irq_disable((uintptr_t)dev->irq_info);
		metal_irq_unregister((uintptr_t)dev->irq_info);
		metal_device_close(dev);
	}
	dev = prproc->ipi_ack_dev;
	if (dev) {
//		metal_irq_disable((uintptr_t)dev->irq_info);
//		metal_irq_unregister((uintptr_t)dev->irq_info);
		metal_device_close(dev);
	}
#endif /* !RPMSG_NO_IPI */
	if (prproc->shm_dev)
		metal_device_close(prproc->shm_dev);
}

static void *
sifive_u_linux_proc_mmap(struct remoteproc *rproc, metal_phys_addr_t *pa,
              metal_phys_addr_t *da, size_t size,
              unsigned int attribute, struct metal_io_region **io)
{
    struct remoteproc_priv *prproc;
    metal_phys_addr_t lpa, lda;
    struct metal_io_region *tmpio;

    (void)attribute;
    (void)size;
    if (!rproc)
        return NULL;
    prproc = rproc->priv;
    lpa = *pa;
    lda = *da;

    if (lpa == METAL_BAD_PHYS && lda == METAL_BAD_PHYS)
        return NULL;
    if (lpa == METAL_BAD_PHYS)
        lpa = lda;
    if (lda == METAL_BAD_PHYS)
        lda = lpa;
    tmpio = prproc->shm_io;
    if (!tmpio)
        return NULL;

    *pa = lpa;
    *da = lda;
    if (io)
        *io = tmpio;
    return metal_io_phys_to_virt(tmpio, lpa);
}

static int sifive_u_linux_proc_notify(struct remoteproc *rproc, uint32_t id)
{
	struct remoteproc_priv *prproc;
#ifndef RPMSG_NO_IPI
	static unsigned int com_reg_val = 0x1; /* COM_REG value to write */
#endif

	(void)id;
	if (!rproc)
		return -1;
	prproc = rproc->priv;

#ifdef RPMSG_NO_IPI
	metal_io_write32(prproc->shm_poll_io, 0, POLL_STOP);
#else /* RPMSG_NO_IPI */
	atomic_int ipi_nokick;

	metal_io_write32(prproc->ipi_io, BX2_MSS_MCCI_COM_REG0_ADDR,
			com_reg_val++);

#if 0
	do {
		ipi_nokick = atomic_load(&ipc_nokick);
	} while (ipi_nokick != 0);
#endif
#endif /* !RPMSG_NO_IPI */
	return 0;
}

/* processor operations from r5 to a53. It defines
 * notification operation and remote processor managementi operations. */
struct remoteproc_ops sifive_u_linux_proc_ops = {
    .init = sifive_u_linux_proc_init,
    .remove = sifive_u_linux_proc_remove,
    .mmap = sifive_u_linux_proc_mmap,
    .notify = sifive_u_linux_proc_notify,
    .start = NULL,
    .stop = NULL,
    .shutdown = NULL,
};
