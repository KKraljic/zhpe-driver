SHELL := /bin/bash
MAKEFILE_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
KVERSION := $(shell uname -r)
RHEL := $(shell !(lsb_release -is | grep -qi 'centos\|redhat' ) || \
	  echo "-DHAVE_RHEL" )

obj-m += zhpe_offloaded.o
zhpe_offloaded-objs += zhpe_offloaded_core.o zhpe_offloaded_uuid.o zhpe_offloaded_zmmu.o zhpe_offloaded_memreg.o zhpe_offloaded_pasid.o zhpe_offloaded_queue.o zhpe_offloaded_rkey.o zhpe_offloaded_msg.o zhpe_offloaded_intr.o

ccflags-y += -I$ $(src)/include -Wno-date-time -mpreferred-stack-boundary=4
ccflags-y += $(RHEL)

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
