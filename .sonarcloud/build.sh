#!/bin/bash

sed -Ei "s/#define HARDWARE_VERSION [0-9]+/#define HARDWARE_VERSION ${HARDWARE_VERSION}/" includes/config.h
sed -Ei "s/#define MODE .+/#define MODE ${MODE}/" includes/config.h
arduino-cli compile --build-path "$PWD/build/$SOFTWARE_FULLNAME" --fqbn STM32:stm32:Nucleo_64:opt=o3std,pnum=NUCLEO_F411RE --verbose srcs/respirator.cpp --output "output/$SOFTWARE_FULLNAME"
