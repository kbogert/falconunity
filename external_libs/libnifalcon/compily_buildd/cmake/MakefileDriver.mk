######################################################################################
# CMake Driver Makefile
######################################################################################

BUILD_TARGET_SCRIPT := $(dir $(abspath $(MAKEFILE_LOCATION)))/../python/build_targets.py
LOCAL_BUILD_TARGET := $(shell $(BUILD_TARGET_SCRIPT) -l)
BUILD_DOC_SCRIPT := $(dir $(abspath $(MAKEFILE_LOCATION)))/../python/build_doc.py

TEST_RUNNER=../testing/runtests.py
FIND_TEST_RUNNER = $(if $(wildcard $(TEST_RUNNER)), $(info Found test runner), \
	$(error Unable to find $(TEST_RUNNER) script, make sure 'testing' is checked out))

.PHONY : tests testdoc clean-all all

all:
	@echo -e  "\nAvailable build targets:"
#	@echo -e  "\tlocal     - Build for the local architecture"
	@echo -e  "\ttests     - Run the automatic test runner"
	@echo -e  "\ttestdoc   - Generate test plan document"
#	@echo -e  "\tclean-all - Clean all build directories"
	@echo -e  ""
	@echo -e  "Additional arguments to build targets:"
	@echo -e  "\tBuild Types:" 
	@echo -e  "\t\trelease"
	@echo -e  "\t\trelwithdebinfo"
	@echo -e  "\t\tdebug"
	@echo -e  ""
	@echo -e  "\tLibrary Types"
	@echo -e  "\t\tstatic"
	@echo -e  "\t\tshared"
	@echo -e  ""
	@echo -e  "\t\tBuilt Types"
	@echo -e  "\t\tdistcc (OS X only)"
	@echo -e  
	@echo -e  "Format:"
	@echo -e  "\t[target]_[build type]_[library type]"
	@echo -e  "\tex."
	@echo -e  "\t\tdarwin_10.5_x86_release_shared_distcc"
	@echo -e  "\tex."
	@echo -e  "\t\twindows_mingw4_x86_debug_static"
	@echo -e  ""
	@echo -e  "Valid build targets:"
	@echo -e  $(foreach target,$(shell $(BUILD_TARGET_SCRIPT) -a), "\t$(target)\n")


doc-gen:
	@echo "Building documentation for project..."
	$(BUILD_DOC_SCRIPT) -d `pwd`/doc -o `pwd`/build_doc

local: $(LOCAL_BUILD_TARGET)
	@echo "Setup build system for local target $(LOCAL_BUILD_TARGET)"

tests:
	$(FIND_TEST_RUNNER)
	@echo "Running all tests.."
	@../testing/runtests.py

testdoc:
	$(FIND_TEST_RUNNER)
	@echo "Generating testplan: testplan.html"
	@../testing/runtests.py --generate-doc > testplan.html

%:
	@$(BUILD_TARGET_SCRIPT) -c $*
	@echo "Setting up build directory for $*"
	mkdir -p $(shell $(BUILD_TARGET_SCRIPT) -b $*)
	cd $(shell $(BUILD_TARGET_SCRIPT) -b $*) ; $(shell $(BUILD_TARGET_SCRIPT) -d $*) cmake -G $(shell $(BUILD_TARGET_SCRIPT) -g $*) $(CMAKE_DEFINES) -DBUILDSYS_BUILD_PLATFORM=$(shell $(BUILD_TARGET_SCRIPT) -i $*) -DBUILDSYS_BUILD_DIR=$(shell $(BUILD_TARGET_SCRIPT) -b $*) $(shell $(BUILD_TARGET_SCRIPT) -e $*) ..

#clean-all:
#	rm -rf $(foreach target,$(shell $(BUILD_TARGET_SCRIPT) -a), $(shell $(BUILD_TARGET_SCRIPT) -b $(target))*)
