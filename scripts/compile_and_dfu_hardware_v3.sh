#!/bin/bash

##
#  compile_and_dfu_hardware_v3.sh
#  Compile & DFU HW3 utility
#
#  Copyright 2021, Makers For Life
#  Author: Makers For Life
##

arduino-cli compile --fqbn STM32:stm32:Nucleo_64:pnum=NUCLEO_F411RE ../srcs/respirator.cpp --output-dir ../builds || exit 1

dfu-util -a 0 --dfuse-address 0x08000000 -D builds/srcs.ino.bin || exit 1

exit 0
