######################################################################################
# CMake Driver Makefile
######################################################################################

BUILD_TARGET_SCRIPT := $(dir $(abspath $(MAKEFILE_LOCATION)))/build_targets.py
BUILD_DOC_SCRIPT := $(dir $(abspath $(MAKEFILE_LOCATION)))/build_doc.py
LOCAL_BUILD_TARGET := $(shell $(BUILD_TARGET_SCRIPT) -l)

.PHONY : clean-all all

all:
	@echo  "\nAvailable build targets:"
	@echo  "\tlocal     - Build for the local architecture"
	@echo  "\tclean-all - Clean all build directories"
	@echo  "\tdoc-gen   - Generate documentation (asciidoc and doxygen)"
	@echo  "Valid build targets:"
	@echo  $(foreach target,$(shell $(BUILD_TARGET_SCRIPT) -a), "\t$(target)\n")

local: $(LOCAL_BUILD_TARGET)
	@echo "Setup build system for local target $(LOCAL_BUILD_TARGET)"

doc-gen:
	@echo "Building documentation for project..."
	$(BUILD_DOC_SCRIPT) -d `pwd`/../doc -o `pwd`/build_doc

%:
	@$(BUILD_TARGET_SCRIPT) -c $*
	@echo "Setting up build directory for $*"
	mkdir -p $(shell $(BUILD_TARGET_SCRIPT) -b $*)
	cd $(shell $(BUILD_TARGET_SCRIPT) -b $*) ; cmake -G $(shell $(BUILD_TARGET_SCRIPT) -g $*) -DNP_BUILD_PLATFORM=$* $(shell $(BUILD_TARGET_SCRIPT) -e $*) ../..

clean-all:
	rm -rf $(foreach target,$(shell $(BUILD_TARGET_SCRIPT) -a), $(shell $(BUILD_TARGET_SCRIPT) -b $(target)))

#all: 
#	@echo Specify target:
#	@echo make osx_10.5
#	@echo   - build for OSX 10.5 : gcc 4.0
#	@echo make mingw_3_win32
#	@echo   - build Windows 32 or 64 bit, XP/Vista : MinGW \(expects c:/MinGW, "Official" 3.4.5\)
#	@echo make mingw_4_win32
#	@echo   - build Windows 32 or 64 bit, XP/Vista : MinGW \(expects c:/MinGW, TDM based gcc 4.3+\) - PREFERRED CHOICE FOR WINDOWS BUILDING
#	@echo make mingw_3_cc
#	@echo   - build crosscompiled mingw 3 : MinGW \("Official" 3.4.5, deprecated\), expects crosscompiler in /usr/local/i386-mingw32-3.4.5
#	@echo make mingw_4_cc
#	@echo   - build crosscompiled mingw 4 : expects crosscompiler in /usr/local/i386-mingw32msvc - PREFERRED CHOICE FOR WINDOWS BUILDING
#	@echo   - expects crosscompiler build from Makefile at http://www.profv.de/mingw_cross_env/README.html
#	@echo make linux_debian_x86
#	@echo   - build 32-bit Debian 4.0 : gcc 4, forces 32-bit on 64-bit platforms
#	@echo make linux_debian_x86_64
#	@echo   - build 64-bit Debian 4.0 : gcc 4 \(Not cross compiling, expects at 64-bit system\)
#	@echo make eldk-ppc_4xxFP
#	@echo   - build cross compiled ELDK based PPC binaries
#
#linux_debian_x86:
#	mkdir -p build_linux_debian_x86
#	cd build_linux_debian_x86; cmake -G "Unix Makefiles" -DFIVETEN_BUILD_PLATFORM=linux_debian_x86 -DFORCE_32_BIT=TRUE ..
#linux_debian_x86_64:
#	mkdir -p build_linux_debian_x86_64
#	cd build_linux_debian_x86_64; cmake -G "Unix Makefiles" -DFIVETEN_BUILD_PLATFORM=linux_debian_x86_64 ..
#osx_10.5:
#	mkdir -p build_osx_10.5
#	cd build_osx_10.5;	cmake -G "Unix Makefiles" -DFIVETEN_BUILD_PLATFORM=osx_10.5 ..
#mingw_3_win32:
#	mkdir -p build_mingw_3
#	cd build_mingw_3;	cmake -G "MSYS Makefiles" -DFIVETEN_BUILD_PLATFORM=mingw_3 .. 
#mingw_4_win32:
#	mkdir -p build_mingw_4
#	cd build_mingw_4;	cmake -G "MSYS Makefiles" -DFIVETEN_BUILD_PLATFORM=mingw_4 .. 
#mingw_3_cc:
#	mkdir -p build_mingw_3
#	cd build_mingw_3; cmake -G "Unix Makefiles" -DFIVETEN_BUILD_PLATFORM=mingw_3 -DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/mingw_3_toolchain.cmake ..
#mingw_4_cc:
#	mkdir -p build_mingw_4
#	cd build_mingw_4; cmake -G "Unix Makefiles" -DFIVETEN_BUILD_PLATFORM=mingw_4 -DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/mingw_4_toolchain.cmake ..
#eldk-ppc_4xxFP:
#	mkdir -p build_eldk-4.1-ppc_4xxFP
#	cd build_eldk-4.1-ppc_4xxFP; cmake -G "Unix Makefiles" -DFIVETEN_BUILD_PLATFORM=eldk-4.1-ppc_4xxFP -DCMAKE_TOOLCHAIN_FILE=../../build_sys/cmake/toolchains/eldk_4.1_ppc-4xxFP_toolchain.cmake ..
#clean-all:
#	rm -rf build_linux_debian_x86
#	rm -rf build_linux_debian_x86_64
#	rm -rf build_osx_10.5
#	rm -rf build_mingw_3_win32
#	rm -rf build_mingw_4_win32
#	rm -rf build_mingw_3_cc
#	rm -rf build_mingw_4_cc
#	rm -rf build_eldk-4.1-ppc_4xxFP
