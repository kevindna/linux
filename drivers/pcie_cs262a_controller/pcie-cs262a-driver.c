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

#include "../pci/pci.h"
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

#define CS262_RC_NUM_MSI_IRQS		128

/* ECAM definitions */
#define ECAM_BUS_NUM_SHIFT		20
#define ECAM_DEV_NUM_SHIFT		12



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
	unsigned long msi_map[BITS_TO_LONGS(CS262_RC_NUM_MSI_IRQS)];	// Do we need this?
	struct mutex map_lock;
	struct irq_domain *msi_domain;
	struct irq_domain *leg_domain;
	struct list_head resources;
};



/**
 * xilinx_pcie_map_bus - Get configuration base
 * @bus: PCI Bus structure
 * @devfn: Device/function
 * @where: Offset from base
 *
 * Return: Base address of the configuration space needed to be
 *	   accessed.
 */
static void __iomem *cs262_pcie_map_bus(struct pci_bus *bus,
					 unsigned int devfn, int where)
{
	struct cs262_pcie_port *port = bus->sysdata;
	int relbus;

	dev_warn(&bus->dev, "Sysdata: %d", bus->sysdata);
	//if (!xilinx_pcie_valid_device(bus, devfn))
	//	return NULL;

	relbus = (bus->number << ECAM_BUS_NUM_SHIFT) |
		 (devfn << ECAM_DEV_NUM_SHIFT);


	dev_warn(&bus->dev, "Sysdata: %d  relbus: %d", bus->sysdata, relbus);
	return phys_to_virt(0x40000000);
//	return port->reg_base + relbus + where;
}


/* PCIe operations */
static struct pci_ops cs262_pcie_ops = {
  .map_bus = cs262_pcie_map_bus,
  .read = pci_generic_config_read,
  .write  = pci_generic_config_write,
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
static inline void root_complex_write(struct cs262_pcie_port *port, u32 val, u32 reg) {
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

	//int val = readl(0x400000000);

	dev_warn(dev, "(cs262_pcie_init_port) Physical: %llx, Virtual: %px\n\n", 0x40000000, phys_to_virt(0x40000000));

	int val = 0x40000000;
	int val2 = readl(phys_to_virt(0x40000000));

	dev_warn(dev, "(cs262_pcie_init_port) Direct read: %d, Read Virtual: %d\n\n", val, val2);

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
//	struct device *dev = port->dev;
//	struct device_node *node = dev->of_node;
//	struct resource regs;
//	unsigned int irq;
//	int err;
//
//	err = of_address_to_resource(node, 0, &regs);
//	if (err) {
//		dev_err(dev, "missing \"reg\" property\n");
//		return err;
//	}
//
//  port->cntrl_reg_base = ; // TODO: allocated and map control registers; cannot be simple malloc and mmap
//	port->reg_base = devm_pci_remap_cfg_resource(dev, &regs);  // allocates and maps config space
//	if (IS_ERR(port->reg_base))
//		return PTR_ERR(port->reg_base);
//
//	irq = irq_of_parse_and_map(node, 0);
//	err = devm_request_irq(dev, irq, cs262_pcie_intr_handler,
//			       IRQF_SHARED | IRQF_NO_THREAD,
//			       "xilinx-pcie", port);
//	if (err) {
//		dev_err(dev, "unable to request irq %d\n", irq);
//		return err;
//	}

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

	//printk(KERN_EMERG, "CS262 Driver Probe function called %s\n\n",0);
	dev_warn(dev, "CS262 Driver Probe function called\n\n");

	if (!dev->of_node)
		return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*port));
	if (!bridge)
		return -ENODEV;

	dev_warn(dev, "[cs262_pcie_probe] brigde pointer: %d -- Physical Address: %d\n\n", bridge, virt_to_phys(bridge));

	port = pci_host_bridge_priv(bridge);
	mutex_init(&port->map_lock);
	port->dev = dev;

	// err = cs262_pcie_parse_dt(port);
	// if (err) {
	// 	dev_err(dev, "Parsing DT failed\n");
	// 	return err;
	// }

	cs262_pcie_init_port(port);

	//err = cs262_pcie_init_irq_domain(port);
	//if (err) {
	//	dev_err(dev, "Failed creating IRQ Domain\n");
	//	return err;
	//}

	bridge->sysdata = port;
	bridge->ops = &cs262_pcie_ops;

	// err = pci_host_probe(bridge);
	// if (err)
	// 	cs262_free_msi_domains(port);



	err = pci_scan_root_bus_bridge(bridge);
	dev_warn(dev, "CS262 PCIE PROBE: %d (err) \n\n", err);

	if (err < 0)
		return err;

 	// printk(KERN_EMERG, "CS262 PCIE PROBE: %d (err)\n\n", err);


	return err;
}
EXPORT_SYMBOL_GPL(cs262_pcie_probe);


/*
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

//static hello_kevin_ABC(


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


