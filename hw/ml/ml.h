#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qapi/visitor.h"

#ifndef HW_ML_H
#define HW_ML_H

#define PCI_VENDOR_ID_ML 0x4396
#define PCI_DEVICE_ID_ML 0x6666

#define ML(obj)         OBJECT_CHECK(MLState, obj, "ml")

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096

typedef struct {
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define ML_STATUS_COMPUTING    0x01
#define ML_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define ML_DMA_RUN             0x1
#define ML_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define ML_DMA_FROM_PCI       0
# define ML_DMA_TO_PCI         1
#define ML_DMA_IRQ             0x4
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    QEMUTimer dma_timer;
    char dma_buf[DMA_SIZE];
    uint64_t dma_mask;
} MLState;

#endif /* HW_ML_H */
