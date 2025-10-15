SHELL := /bin/bash
MW ?= "CYBERRT"
PROJECT_NAME ?= "ad_simulation"
TARGET_PALTFORM ?= "orin-dev"
TGZ_VER := 1.$$(date '+%Y%m%d-%H%M%S')-$$(git rev-parse --short=5 HEAD).$(PROJECT_NAME)-$(TARGET_PALTFORM)
CLEAN_VER := $$([[ -n $$(git status -s) ]] && echo '-dirty' || echo '-new')
JOBS ?= 8
RELEASE_CONF := release.conf.json

COLOR_RED := \033[1;31m
COLOR_GREEN := \033[1;32m
COLOR_YELLOW := \033[1;33m
COLOR_RESET := \033[0m

FORCE:

all: $(RELEASE_CONF)
	@mkdir -p build
	@echo ""
	@echo "###################################################"
	@echo "# CMake"
	@echo "###################################################"
	@echo ""
	@cd build && cmake ..
	@echo ""
	@echo "###################################################"
	@echo "# Build"
	@echo "###################################################"
	@echo ""
	@cd build && make --no-print-directory -j$(JOBS)

clean:
	@rm -r build
	@rm -f $(RELEASE_CONF)

# 生成 release.conf.json
$(RELEASE_CONF): FORCE
	@echo "Generating $@"
	@chmod +x resource/tar_extra/release_helper.sh
	@PROJECT_NAME="$(PROJECT_NAME)" TARGET_PALTFORM="$(TARGET_PALTFORM)" ./resource/tar_extra/release_helper.sh

tgz_all: $(RELEASE_CONF)
	@echo "###################################################"
	@echo "# Make tgz_all"
	@echo "###################################################"
	@tar -czvf $(PROJECT_NAME)-$(TARGET_PALTFORM).tar.gz *

tgz_orin: $(RELEASE_CONF)
	@echo "###################################################"
	@echo "# Make tgz_orin"
	@echo "###################################################"
	@mkdir -p build
	@cd build && cmake .. \
		-DCMAKE_TOOLCHAIN_FILE=/home/cmake/orin-toolchain.cmake \
		-DCMAKE_MW="$(MW)" \
		-DUSE_CROSS_COMPILE=1 \
		-DCMAKE_BUILD_TYPE="Release" \
		-DCMAKE_INSTALL_PREFIX="/opt/tmp/ad_simulation" \
		-DMODULE_VERSION="$(TGZ_VER)$(CLEAN_VER)"
	@cd build && make package --no-print-directory -j$(JOBS)

tgz_x86: $(RELEASE_CONF)
	@echo "###################################################"
	@echo "# Make tgz_x86"
	@echo "###################################################"
	@echo ""
	@mkdir -p build
	@cd build && cmake .. \
		-DCMAKE_TOOLCHAIN_FILE=/home/cmake/x86-toolchain.cmake \
		-DCMAKE_BUILD_TYPE="Release" \
		-DCMAKE_INSTALL_PREFIX="/opt/tmp/ad_simulation" \
		-DCATKIN_BUILD_BINARY_PACKAGE="1" \
		-DBUILD_x86_64=ON \
		-DMODULE_VERSION="$(TGZ_VER)$(CLEAN_VER)"
	@cd build && make package --no-print-directory -j$(JOBS)
