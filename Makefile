obj-m += bmm050_driver.o
obj-m += bmm050.o

KERNEL_SRC = /dev/null

all:
		make -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
		make -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
		make -C $(KERNEL_SRC) M=$(PWD) clean
