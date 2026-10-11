.DEFAULT_GOAL := all

all:
	$(MAKE) -C template

clean:
	$(MAKE) -C template clean

program-dfu:
	$(MAKE) -C template program-dfu

libs:
	$(MAKE) -C lib/libDaisy -j4
	$(MAKE) -C lib/DaisySP -j4

clean-libs:
	$(MAKE) -C lib/libDaisy clean
	$(MAKE) -C lib/DaisySP clean

.PHONY: all clean program-dfu libs clean-libs

