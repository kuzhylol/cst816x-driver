obj-m += hynitron-cst816x.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

install: hynitron-cst816x.ko
	mkdir -p /lib/modules/$(shell uname -r)/extra
	install -m 644 hynitron-cst816x.ko /lib/modules/$(shell uname -r)/extra/
	depmod -a $(shell uname -r)

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
