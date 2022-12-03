#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "../pci.h"
// Root complex control registers (MMIO)
#define CS262_RC_VERSION        0x00000000    // Placeholder
#define CS262_RC_STATUS         0x00000000    // Placeholder
#define CS262_RC_READ_REG       0x00000000    // Address of register in RC holding result from read request to endpoint

// TODO: Add true address values
// MSI/RC-CPU Interrupt communciation
#define CS262_PCIE_MSI_REG      0x00000000
#define CS262_PCIE_MSI_MASK_REG 0x00000000
#define CS262_RC_INTR_READ      0x00000000    // Constant denotating data from read available in RC
#define CS262_RC_INTR_DMA       0x00000000    // Constant denoting DMA write from endpoint finished

/**
 * struct cs262_pcie_port - PCIe port information
 * @reg_base: IO Mapped Register Base
 * @dev: Device pointer
 * @msi_map: Bitmap of allocated MSIs
 * @map_lock: Mutex protecting the MSI allocation
 * @msi_domain: MSI IRQ domain pointer
 * @leg_domain: Legacy IRQ domain pointer
 * @resources: Bus Resources
 */
// Need our own version of this
struct cs262_pcie_port {    // Driver's internal representation of root complex
	void __iomem *reg_base;          // For PCIe (config space and endpoint)
  void __iomem *cntrl_reg_base;    // Base address for control registers in root complex
	struct device *dev;
	unsigned long msi_map[BITS_TO_LONGS(XILINX_NUM_MSI_IRQS)];	// Do we need this?
	struct mutex map_lock;
	struct irq_domain *msi_domain;
	struct irq_domain *leg_domain;
	struct list_head resources;
};



/*
  * Function reads from control register in root complex. Not for configuration
  * space, nor accesses directed to endpoint (i.e. communication not in PCI
  * address space)
  */
static inline u32 root_complex_read(struct cs262_pcie_port *port, u32 reg) {
  return readl(port->cntrl_reg_base + reg);

  // Use inlined assembly if headers don't support readl/ioread32
  // Take from arch/risc/mmio.h
  /*
  u32 val;
  asm volatile("lw %0, 0(%1)" : "=r" (val) : "r" (port->cntrl_reg_base + reg)));
  return val;
  */
}

/*
  * Function writes to control register in root complex. Not for configuration
  * space, nor accesses directed to endpoint (i.e. communication not in PCI
  * address space)
  */
static inline u32 root_complex_write(struct cs262_pcie_port *port, u32 reg) {
  writel(val, port->cntrl_reg_base + reg);

  // Use inlined assembly if headers don't support writel/iowrite32
  // Take from arch/risc/mmio.h
  /* asm volatile("sw %0, 0(%1)" : : "r" (val), "r" (port->cntrl_reg_base + reg))); */
}

static inline u32 pcie_read(struct cs262_pcie_port *port, u32 reg) {
	return readl(port->reg_base + reg);

  // Use inlined assembly if headers don't support readl/ioread32
  // Take from arch/risc/mmio.h
  /*
  u32 val;
  asm volatile("lw %0, 0(%1)" : "=r" (val) : "r" (port->reg_base + reg)));
  return val;
  */
}

static inline void pcie_write(struct cs262_pcie_port *port, u32 val, u32 reg) {
	writel(val, port->reg_base + reg);

  // Use inlined assembly if headers don't support writel/iowrite32
  // Take from arch/risc/mmio.h
  /* asm volatile("sw %0, 0(%1)" : : "r" (val), "r" (port->reg_base + reg))); */
}

/**
 * cs262_pcie_init_port - Initialize hardware
 * @port: PCIe port information
 */
static void cs262_pcie_init_port(struct cs262_pcie_port *port) {
	struct device *dev = port->dev;
/*
	if (cs262_pcie_link_up(port))
		dev_info(dev, "PCIe Link is UP\n");
	else
		dev_info(dev, "PCIe Link is DOWN\n");
*/

// TODO: Setup any config registers

}



/**
 * cs262_pcie_parse_dt - Parse Device tree
 * @port: PCIe port information
 *
 * Return: '0' on success and error value on failure
 */
static int cs262_pcie_parse_dt(struct cs262_pcie_port *port)
{
	struct device *dev = port->dev;
	struct device_node *node = dev->of_node;
	struct resource regs;
	unsigned int irq;
	int err;

	err = of_address_to_resource(node, 0, &regs);
	if (err) {
		dev_err(dev, "missing \"reg\" property\n");
		return err;
	}

  port->cntrl_reg_base = ; // TODO: allocated and map control registers; cannot be simple malloc and mmap
	port->reg_base = devm_pci_remap_cfg_resource(dev, &regs);  // allocates and maps config space
	if (IS_ERR(port->reg_base))
		return PTR_ERR(port->reg_base);

	irq = irq_of_parse_and_map(node, 0);
	err = devm_request_irq(dev, irq, cs262_pcie_intr_handler,
			       IRQF_SHARED | IRQF_NO_THREAD,
			       "xilinx-pcie", port);
	if (err) {
		dev_err(dev, "unable to request irq %d\n", irq);
		return err;
	}

	return 0;
}

/**
 * cs262_pcie_probe - Probe function
 * @pdev: Platform device pointer
 *
 * Return: '0' on success and error value on failure
 */
static int cs262_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cs262_pcie_port *port;
	struct pci_host_bridge *bridge;
	int err;

	if (!dev->of_node)
		return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*port));
	if (!bridge)
		return -ENODEV;

	port = pci_host_bridge_priv(bridge);
	mutex_init(&port->map_lock);
	port->dev = dev;

	// err = cs262_pcie_parse_dt(port);
	// if (err) {
	// 	dev_err(dev, "Parsing DT failed\n");
	// 	return err;
	// }

	cs262_pcie_init_port(port);

	err = cs262_pcie_init_irq_domain(port);
	if (err) {
		dev_err(dev, "Failed creating IRQ Domain\n");
		return err;
	}

	bridge->sysdata = port;
	bridge->ops = &cs262_pcie_ops;

	err = pci_host_probe(bridge);
	if (err)
		cs262_free_msi_domains(port);

	return err;
}

/**
 * cs262_pcie_intr_handler - Interrupt Service Handler
 * @irq: IRQ number
 * @data: PCIe port information
 *
 * Return: IRQ_HANDLED on success and IRQ_NONE on failure
 */
static irqreturn_t cs262_pcie_intr_handler(int irq, void *data)
{
	struct cs262_pcie_port *port = (struct cs262_pcie_port *)data;
	struct device *dev = port->dev;
	u32 val, mask, status;

	/* Read interrupt decode and mask registers */
	val = pcie_read(port, CS262_PCIE_MSI_REG);
	mask = pcie_read(port, CS262_PCIE_MSI_MASK_REG); // Implementation dependent; may not need

  status = val & mask;

  // TODO: Implement interrupt handling (1) DMA and (2) read req

  if (status & CS262_RC_INTR_DMA) {
    // TODO: Perform some operations
  }

  if (status & CS262_RC_INTR_READ) {
    root_complex_read(port, CS262_RC_READ_REG);
  }

  // TODO: Perform error checks



	return IRQ_HANDLED;
}

// TODO: MSI FUCNTIONS

//--------------------------------------------------//
// Structures
//--------------------------------------------------//
// Need to match driver to device (device tree base)
static const struct of_device_id cs262_pcie_of_match[] = {
  { .compatible = "pcie-cs262a", },
  {}
};

// All drivers need this which registers driver
static struct platform_driver cs262_pcie_driver = {
  .driver = {
    .name = "cs262-pcie",
    .of_match_table = cs262_pcie_of_match,
    .suppress_bind_attrs = true,
  },
  .probe = cs262_pcie_probe, // Must have
};
builtin_platform_driver(cs262_pcie_driver); // Register driver with kernel (underlying platform_register_driver call)


/* PCIe operations */
static struct pci_ops cs262_pcie_ops = {
  .map_bus = cs262_pcie_map_bus,
  .read = pci_generic_config_read,
  .write  = pci_generic_config_write,
};
