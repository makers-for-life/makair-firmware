arduino-cli compile --fqbn STM32:stm32:Nucleo_64:pnum=NUCLEO_F411RE srcs/srcs.cpp --output-dir output
#dfu-util -a 0 --dfuse-address 0x08000000 -D output/srcs.ino.bin