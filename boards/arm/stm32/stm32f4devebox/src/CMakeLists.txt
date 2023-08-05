# ##############################################################################
# boards/arm/stm32/stm32f4discovery/src/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS stm32_boot.c stm32_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_STM32_SDIO)
  list(APPEND SRCS stm32_sdio.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS stm32_reset.c)
  endif()
endif()

if(CONFIG_LCD_ILI9341)
  list(APPEND SRCS stm32_ili9341.c)
endif

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if(CONFIG_INPUT)
  list(APPEND SRCS stm32_xpt2046.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS stm32_timer.c)
endif()

target_sources(board PRIVATE ${SRCS})

# TODO: make this the default and then allow boards to redefine
set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")

# TODO:move this to apropriate arch/toolchain level
set_property(
  GLOBAL APPEND
  PROPERTY COMPILE_OPTIONS $<$<CONFIG:Release>:-fno-strict-aliasing
           -fomit-frame-pointer>)

# TODO: see where to put pic flags set_property(TARGET nuttx APPEND PROPERTY
# NUTTX_COMPILE_OPTIONS $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:-fpic -msingle-pic-base
# -mpic-register=r10>)

# ifeq ($(CONFIG_ARMV7M_TOOLCHAIN_CLANGL),y) ARCHCFLAGS += -nostdlib
# -ffreestanding ARCHCXXFLAGS += -nostdlib -ffreestanding else ARCHCFLAGS +=
# -funwind-tables ARCHCXXFLAGS += -fno-rtti -funwind-tables ifneq
# ($(CONFIG_DEBUG_NOOPT),y) ARCHOPTIMIZATION += -fno-strength-reduce endif endif

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_COMPILE_OPTIONS -funwind-tables)
set_property(GLOBAL APPEND PROPERTY COMPILE_OPTIONS -fno-strength-reduce)

# TODO: nxflat NXFLATLDFLAGS1 = -r -d -warn-common NXFLATLDFLAGS2 =
# $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld
# -no-check-sections LDNXFLATFLAGS = -e main -s 2048

# Loadable module definitions

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -mlong-calls)
set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_MODULE_LINK_OPTIONS -r -e module_initialize -T
           ${NUTTX_DIR}/libs/libc/modlib/gnu-elf.ld)

# ELF module definitions

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_APP_COMPILE_OPTIONS -mlong-calls)
set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_APP_LINK_OPTIONS -r -e main
           -T${NUTTX_BOARD_DIR}/scripts/gnu-elf.ld)
