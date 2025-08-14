#
# Copyright 2024 Xilinx, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

ECHO=@echo
RM = rm -f
RMDIR = rm -rf

############################## Setup and Variables ##############################

# Build Target: sw_emu, hw_emu, or hw
# Default to sw_emu build if not specified
TARGET ?= sw_emu

# Target Platform (Alveo U250)
PLATFORM ?= xilinx_u250_gen3x16_xdma_4_1_202210_1
#PLATFORM ?= xilinx_u55c_gen3x16_xdma_3_202210_1

# Kernel Name
KERNEL_NAME := subgraphIsomorphism

############################## Build Directories ##############################

TEMP_DIR := ./_x.$(TARGET).$(PLATFORM)
BUILD_DIR := ./build_dir.$(TARGET).$(PLATFORM)
PACKAGE_OUT = ./package.$(TARGET)

############################## Output Files ##############################

# Project Specifics
EXECUTABLE := ./subiso_test_host
HOST_SRC := ./source/subiso-test.cpp
KERNEL_SRC := ./source/subgraphIsomorphism.cpp
KERNEL_XO := $(BUILD_DIR)/$(KERNEL_NAME).xo

# Kernel Linking Output
LINK_OUTPUT := $(BUILD_DIR)/$(KERNEL_NAME).link.xclbin
XCLBIN_FILE := $(BUILD_DIR)/$(KERNEL_NAME).xclbin

# Kernel HLS & Link Configuration File
KERNEL_CFG := ./subiso.cfg

############################## Source Files ##############################

# Host source files
HOST_SRCS := source/subiso-test.cpp source/cmdlineparser.cpp source/logger.cpp

############################## Vitis Compiler and Linker ##############################

# V++ Compiler
VPP := v++

# Kernel Compiler Flags (V++) for C++ to XO compilation
VPP_FLAGS := --save-temps -t $(TARGET) --platform $(PLATFORM) --temp_dir $(TEMP_DIR)
VPP_FLAGS += --config $(KERNEL_CFG)

# Kernel debug flag
VPP_FLAGS += -DDEBUG_INTERFACE=1

# Set HLS synthesis target frequency
VPP_FLAGS += --hls.clock 300000000:$(KERNEL_NAME)

# Kernel Linker Flags (V++)
VPP_LDFLAGS += -g

############################## Host Compiler ##############################

# C++ Compiler
CXX := g++

# CXX Flags
CXXFLAGS := -I./source \
            -I$(XILINX_XRT)/include \
            -I$(XILINX_VIVADO)/include \
            -Wall -O3 -g -std=c++17 \
            -fmessage-length=0 -DXILINX_XRT

CXXFLAGS += -DDEBUG_INTERFACE=1

# LDFLAGS
LDFLAGS := -L$(XILINX_XRT)/lib -lOpenCL -lpthread -lrt -lstdc++ -luuid -lxrt_coreutil

############################## Execution Arguments ##############################

# Command Line Arguments for Running the Host Executable
CMD_ARGS = -x $(XCLBIN_FILE)

# Emulation Configuration Directory
EMCONFIG_DIR = $(TEMP_DIR)

############################## Build Rules ##############################

.PHONY: all clean cleanall docs emconfig help host build xclbin run test check-platform check-device check-vitis check-xrt kernel_compile

# Default target
all: check-platform check-device check-vitis $(EXECUTABLE) $(XCLBIN_FILE) emconfig

host: $(EXECUTABLE)

kernel_compile: $(KERNEL_XO)

# Build xclbin (depends on the kernel .xo existing)
build: check-vitis check-device $(XCLBIN_FILE)

xclbin: build

# Kernel Build Rules

# 1. Compile Kernel (.cpp -> .xo)
$(KERNEL_XO): $(KERNEL_SRC) | check-vitis
	$(ECHO) "Compiling kernel source $< to XO file $@..."
	mkdir -p $(BUILD_DIR)
	$(VPP) $(VPP_FLAGS) --compile -k $(KERNEL_NAME) -o $@ $<

# 2. Link Kernel (.xo -> .xclbin)
$(XCLBIN_FILE): $(KERNEL_XO) | check-vitis
	$(ECHO) "Linking kernel object $< to create XCLBIN file..."
	mkdir -p $(BUILD_DIR)
	$(VPP) --link $(VPP_FLAGS) $(VPP_LDFLAGS) --kernel $(KERNEL_NAME) -o'$(LINK_OUTPUT)' $<
	$(ECHO) "Packaging $(XCLBIN_FILE)..."
	$(VPP) --package $(VPP_FLAGS) $(LINK_OUTPUT) --package.out_dir $(PACKAGE_OUT) -o $@

# Host Build Rule

# 1. Compile Host Executable
$(EXECUTABLE): $(HOST_SRCS) | check-xrt
	$(ECHO) "Compiling host application..."
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

# Emulation and Execution Rules

# Generate emulation configuration
emconfig: $(EMCONFIG_DIR)/emconfig.json

$(EMCONFIG_DIR)/emconfig.json: check-platform
	$(ECHO) "Generating emulation configuration file..."
	mkdir -p $(EMCONFIG_DIR)
	emconfigutil --platform $(PLATFORM) --od $(EMCONFIG_DIR)

# Run the application
run: all
	$(ECHO) "Running application..."
ifeq ($(TARGET),$(filter $(TARGET),sw_emu hw_emu))
	cp -f $(EMCONFIG_DIR)/emconfig.json .
	XCL_EMULATION_MODE=$(TARGET) ./$(EXECUTABLE) $(CMD_ARGS) $(EXTRA_ARGS)
else
	./$(EXECUTABLE) $(CMD_ARGS) $(EXTRA_ARGS)
endif

# Rule to run for test (similar to run)
test: all
	$(ECHO) "Running test..."
ifeq ($(TARGET),$(filter $(TARGET),sw_emu hw_emu))
	cp -f $(EMCONFIG_DIR)/emconfig.json .
	XCL_EMULATION_MODE=$(TARGET) ./$(EXECUTABLE) $(CMD_ARGS) $(EXTRA_ARGS)
else
	./$(EXECUTABLE) $(CMD_ARGS) $(EXTRA_ARGS)
endif

############################## Cleanup ##############################

clean:
	-$(RMDIR) $(EXECUTABLE) $(BUILD_DIR) $(TEMP_DIR) ./package.* ./source/*.o
	-$(RMDIR) profile_* TempConfig system_estimate.xtxt *.rpt *.csv *.runtime_summary *.launch_summary *.summary *.json *.log *.jou *.wcfg *.wdb *.html *.xo
	-$(RMDIR) *v++* .Xil dltmp* xmltmp* _x* xvlog* webtalk* .ipcache .Xil vivado*

cleanall:
	$(MAKE) clean
	-$(RMDIR) *xclbin.run_summary qemu-memory-_* emulation _vimage pl* start_simulation.sh *.xclbin

############################## Utility Targets ##############################

check-vitis:
ifndef XILINX_VITIS
	$(error XILINX_VITIS variable is not set, please set it using "source <Vitis_install_path>/settings64.sh")
endif

check-xrt:
ifndef XILINX_XRT
	$(error XILINX_XRT variable is not set, please set it using "source /opt/xilinx/xrt/setup.sh")
endif

check-platform:
ifndef PLATFORM
	$(error PLATFORM not set. Please set the PLATFORM properly and rerun. Run "make help" for more details.)
endif

check-device:
	@platforminfo -p $(PLATFORM) > /dev/null

help:
	$(ECHO) "Makefile Usage:"
	$(ECHO) "  make all TARGET=<sw_emu/hw_emu/hw> PLATFORM=<FPGA platform>"
	$(ECHO) "      Command to generate the design for specified Target and Platform."
	$(ECHO) ""
	$(ECHO) "  make build TARGET=<sw_emu/hw_emu/hw> PLATFORM=<FPGA platform>"
	$(ECHO) "      Command to build the xclbin and host application."
	$(ECHO) ""
	$(ECHO) "  make xclbin TARGET=<sw_emu/hw_emu/hw> PLATFORM=<FPGA platform>"
	$(ECHO) "      Command to build the xclbin application (links existing XO)."
	$(ECHO) ""
	$(ECHO) "  make host"
	$(ECHO) "      Command to build the host application."
	$(ECHO) ""
	$(ECHO) "  make run TARGET=<sw_emu/hw_emu/hw> PLATFORM=<FPGA platform> [EXTRA_ARGS=\"<host_app_args>\"]"
	$(ECHO) "      Command to run the application. Use EXTRA_ARGS for host args."
	$(ECHO) ""
	$(ECHO) "  make test TARGET=<sw_emu/hw_emu/hw> PLATFORM=<FPGA platform> [EXTRA_ARGS=\"<host_app_args>\"]"
	$(ECHO) "      Command to run the application for testing."
	$(ECHO) ""
	$(ECHO) "  make clean"
	$(ECHO) "      Command to remove intermediate files."
	$(ECHO) ""
	$(ECHO) "  make cleanall"
	$(ECHO) "      Command to remove all generated files."
	$(ECHO) ""
