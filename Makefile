# Get the current date and time in the format YYYYMMDD_HHMMSS
VERSION_STRING := $(shell date +"%Y%m%d_%H%M%S")
CFLAGS ?=
CFLAGS += -Wno-address-of-packed-member -DVERSION_STRING="\"$(VERSION_STRING)\""

SRCS :=msp_parser.cpp
OUTPUT ?= $(PWD)
BUILD = $(CXX) $(SRCS) -I $(SDK)/include -I$(TOOLCHAIN)/usr/include -I$(PWD) -L$(DRV) $(CFLAGS) $(LIB) -Os -s $(CFLAGS) -o $(OUTPUT)

VERSION := $(shell git describe --always --dirty)

version.h:
	echo "Git version: $(VERSION)"
	echo "#ifndef VERSION_H" > version.h
	echo "#define VERSION_H" >> version.h
	echo "#define GIT_VERSION \"$(VERSION)\"" >> version.h
	echo "#endif // VERSION_H" >> version.h

all: version.h

clean:
	rm -f *.o msp_parser

goke: version.h
	$(eval SDK = ./sdk/gk7205v300)
	$(eval CFLAGS += -D__GOKE__)
	$(eval LIB = -ldl)
	$(BUILD)

hisi: version.h
	$(eval SDK = ./sdk/hi3516ev300)
	$(eval CFLAGS += -D__GOKE__)
	$(eval LIB = -lmpi -lsecurec)
	$(BUILD)

star6b0: version.h
	$(eval SDK = ./sdk/infinity6)
	$(eval CFLAGS += -D__SIGMASTAR__ -D__INFINITY6__ -D__INFINITY6B0__)
	$(eval LIB = -lm -lmi_rgn -lmi_sys)
	$(BUILD)

star6e: version.h
	$(eval SDK = ./sdk/infinity6)
	$(eval CFLAGS += -D__SIGMASTAR__ -D__INFINITY6__ -D__INFINITY6E__)
	$(eval LIB = -lm)
	$(BUILD)

native: version.h
	$(eval SDK = ./sdk/gk7205v300)
	$(eval CFLAGS += -D_x86)
	$(eval LIB =  -lm)
	$(eval BUILD = $(CXX) $(SRCS) -I $(SDK)/include -L $(DRV) $(CFLAGS) $(LIB) -O0 -g -o $(OUTPUT))
	$(BUILD)

rockchip: version.h
	$(eval SDK = ./sdk/gk7205v300)
	$(eval CFLAGS += -D__ROCKCHIP__)
	$(eval LIB = `pkg-config --libs cairo x11` -lm -lrt)
	$(eval BUILD = $(CXX) $(SRCS) -I $(SDK)/include -L $(DRV) $(CFLAGS) $(LIB) -O0 -g -o $(OUTPUT))
	$(BUILD)