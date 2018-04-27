/*
 * QEMU Machine learning device
 */
#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qapi/visitor.h"
#include "ml.h"

static bool ml_msi_enabled(MLState *ml)
{
    return msi_enabled(&ml->pdev);
}

static void ml_raise_irq(MLState *ml, uint32_t val)
{
    ml->irq_status |= val;
    if (ml->irq_status) {
        if (ml_msi_enabled(ml)) {
            msi_notify(&ml->pdev, 0);
        } else {
            pci_set_irq(&ml->pdev, 1);
        }
    }
}

static void ml_lower_irq(MLState *ml, uint32_t val)
{
    ml->irq_status &= ~val;

    if (!ml->irq_status && !ml_msi_enabled(ml)) {
        pci_set_irq(&ml->pdev, 0);
    }
}

static bool within(uint32_t addr, uint32_t start, uint32_t end)
{
    return start <= addr && addr < end;
}

static void ml_check_range(uint32_t addr, uint32_t size1, uint32_t start,
                uint32_t size2)
{
    uint32_t end1 = addr + size1;
    uint32_t end2 = start + size2;

    if (within(addr, start, end2) &&
            end1 > addr && within(end1, start, end2)) {
        return;
    }

    hw_error("ML: DMA range 0x%.8x-0x%.8x out of bounds (0x%.8x-0x%.8x)!",
            addr, end1 - 1, start, end2 - 1);
}

static dma_addr_t ml_clamp_addr(const MLState *ml, dma_addr_t addr)
{
    dma_addr_t res = addr & ml->dma_mask;

    if (addr != res) {
        printf("ML: clamping DMA %#.16"PRIx64" to %#.16"PRIx64"!\n", addr, res);
    }

    return res;
}

static void ml_dma_timer(void *opaque)
{
    MLState *ml = opaque;
    bool raise_irq = false;

    if (!(ml->dma.cmd & ML_DMA_RUN)) {
        return;
    }

    if (ML_DMA_DIR(ml->dma.cmd) == ML_DMA_FROM_PCI) {
        uint32_t dst = ml->dma.dst;
        ml_check_range(dst, ml->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START;
        pci_dma_read(&ml->pdev, ml_clamp_addr(ml, ml->dma.src),
                ml->dma_buf + dst, ml->dma.cnt);
    } else {
        uint32_t src = ml->dma.src;
        ml_check_range(src, ml->dma.cnt, DMA_START, DMA_SIZE);
        src -= DMA_START;
        pci_dma_write(&ml->pdev, ml_clamp_addr(ml, ml->dma.dst),
                ml->dma_buf + src, ml->dma.cnt);
    }

    ml->dma.cmd &= ~ML_DMA_RUN;
    if (ml->dma.cmd & ML_DMA_IRQ) {
        raise_irq = true;
    }

    if (raise_irq) {
        ml_raise_irq(ml, DMA_IRQ);
    }
}

static void dma_rw(MLState *ml, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && (ml->dma.cmd & ML_DMA_RUN)) {
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        timer_mod(&ml->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
}

static uint64_t ml_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MLState *ml = opaque;
    uint64_t val = ~0ULL;

    if (size != 4) {
        return val;
    }

    switch (addr) {
    case 0x00:
        val = 0x010000000;
        break;
    case 0x04:
        val = ml->addr4;
        break;
    case 0x08:
        qemu_mutex_lock(&ml->thr_mutex);
        val = ml->fact;
        qemu_mutex_unlock(&ml->thr_mutex);
        break;
    case 0x20:
        val = atomic_read(&ml->status);
        break;
    case 0x24:
        val = ml->irq_status;
        break;
    case 0x80:
        dma_rw(ml, false, &val, &ml->dma.src, false);
        break;
    case 0x88:
        dma_rw(ml, false, &val, &ml->dma.dst, false);
        break;
    case 0x90:
        dma_rw(ml, false, &val, &ml->dma.cnt, false);
        break;
    case 0x98:
        dma_rw(ml, false, &val, &ml->dma.cmd, false);
        break;
    }

    return val;
}

static void ml_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    MLState *ml = opaque;

    if (addr < 0x80 && size != 4) {
        return;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        return;
    }

    switch (addr) {
    case 0x04:
        ml->addr4 = ~val;
        break;
    case 0x08:
        if (atomic_read(&ml->status) & ML_STATUS_COMPUTING) {
            break;
        }
        /* ML_STATUS_COMPUTING cannot go 0->1 concurrently, because it is only
         * set in this function and it is under the iothread mutex.
         */
        qemu_mutex_lock(&ml->thr_mutex);
        ml->fact = val;
        atomic_or(&ml->status, ML_STATUS_COMPUTING);
        qemu_cond_signal(&ml->thr_cond);
        qemu_mutex_unlock(&ml->thr_mutex);
        break;
    case 0x20:
        if (val & ML_STATUS_IRQFACT) {
            atomic_or(&ml->status, ML_STATUS_IRQFACT);
        } else {
            atomic_and(&ml->status, ~ML_STATUS_IRQFACT);
        }
        break;
    case 0x60:
        ml_raise_irq(ml, val);
        break;
    case 0x64:
        ml_lower_irq(ml, val);
        break;
    case 0x80:
        dma_rw(ml, true, &val, &ml->dma.src, false);
        break;
    case 0x88:
        dma_rw(ml, true, &val, &ml->dma.dst, false);
        break;
    case 0x90:
        dma_rw(ml, true, &val, &ml->dma.cnt, false);
        break;
    case 0x98:
        if (!(val & ML_DMA_RUN)) {
            break;
        }
        dma_rw(ml, true, &val, &ml->dma.cmd, true);
        break;
    }
}

static const MemoryRegionOps ml_mmio_ops = {
    .read = ml_mmio_read,
    .write = ml_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/*
 * We purposely use a thread, so that users are forced to wait for the status
 * register.
 */
static void *ml_fact_thread(void *opaque)
{
    MLState *ml = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&ml->thr_mutex);
        while ((atomic_read(&ml->status) & ML_STATUS_COMPUTING) == 0 &&
                        !ml->stopping) {
            qemu_cond_wait(&ml->thr_cond, &ml->thr_mutex);
        }

        if (ml->stopping) {
            qemu_mutex_unlock(&ml->thr_mutex);
            break;
        }

        val = ml->fact;
        qemu_mutex_unlock(&ml->thr_mutex);

        while (val > 0) {
            ret *= val--;
        }

        /*
         * We should sleep for a random period here, so that students are
         * forced to check the status properly.
         */

        qemu_mutex_lock(&ml->thr_mutex);
        ml->fact = ret;
        qemu_mutex_unlock(&ml->thr_mutex);
        atomic_and(&ml->status, ~ML_STATUS_COMPUTING);

        if (atomic_read(&ml->status) & ML_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            ml_raise_irq(ml, FACT_IRQ);
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void pci_ml_realize(PCIDevice *pdev, Error **errp)
{
    MLState *ml = DO_UPCAST(MLState, pdev, pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    timer_init_ms(&ml->dma_timer, QEMU_CLOCK_VIRTUAL, ml_dma_timer, ml);

    qemu_mutex_init(&ml->thr_mutex);
    qemu_cond_init(&ml->thr_cond);
    qemu_thread_create(&ml->thread, "ml", ml_fact_thread,
                       ml, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&ml->mmio, OBJECT(ml), &ml_mmio_ops, ml,
                    "ml-mmio", 1 << 20);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &ml->mmio);
}

static void pci_ml_uninit(PCIDevice *pdev)
{
    MLState *ml = DO_UPCAST(MLState, pdev, pdev);

    qemu_mutex_lock(&ml->thr_mutex);
    ml->stopping = true;
    qemu_mutex_unlock(&ml->thr_mutex);
    qemu_cond_signal(&ml->thr_cond);
    qemu_thread_join(&ml->thread);

    qemu_cond_destroy(&ml->thr_cond);
    qemu_mutex_destroy(&ml->thr_mutex);

    timer_del(&ml->dma_timer);
}

static void ml_obj_uint64(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    uint64_t *val = opaque;

    visit_type_uint64(v, name, val, errp);
}

static void ml_instance_init(Object *obj)
{
    MLState *ml = ML(obj);

    ml->dma_mask = (1UL << 28) - 1;
    object_property_add(obj, "dma_mask", "uint64", ml_obj_uint64,
                    ml_obj_uint64, NULL, &ml->dma_mask, NULL);
}

static void ml_class_init(ObjectClass *class, void *data)
{
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_ml_realize;
    k->exit = pci_ml_uninit;
    k->vendor_id = PCI_VENDOR_ID_ML;
    k->device_id = PCI_DEVICE_ID_ML;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
}

static void pci_ml_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo ml_info = {
        .name          = "ml",
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(MLState),
        .instance_init = ml_instance_init,
        .class_init    = ml_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&ml_info);
}
type_init(pci_ml_register_types)
