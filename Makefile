obj-m	+= gpio-counter.o

all:
        make -C $(KERNEL_DIR) \
                SUBDIRS=$(shell pwd) modules
clean:
        make -C $(KERNEL_DIR) \
                SUBDIRS=$(shell pwd) clean
