all: pcie-cs262a-driver.o

pcie-cs262a-driver.o :
	gcc pcie-cs262a-driver.c -I/scratch/kevinand/repos/FireMarshal/boards/chipyard/linux/include  -o pcie-cs262a-driver.o


obj-$(CONFIG_PCIE_CS262A) += pcie-cs262a-driver.o
