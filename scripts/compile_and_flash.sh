#!/bin/bash

ABSPATH=$(cd "$(dirname "$0")"; pwd)
BASE_DIR="$ABSPATH/../"

pushd "$BASE_DIR" > /dev/null
  # Acquire serial port first
  SERIAL_PORT=$(arduino-cli board list | grep "Serial Port (USB)" | cut -d ' ' -f 1 | head -n 1)

  if [ ! -z "$SERIAL_PORT" ]; then
    echo "Detected STM32 programmer on serial port: $SERIAL_PORT"
    echo "Will proceed."

    # Hold on a bit (give the user a chance to cancel)
    sleep 1

    # Clear out built code
    echo ">> [1] Clearing old builds..."

    sleep 0.5

    rm -f ./output/* || exit 1

    echo "Old builds cleared."

    # Compile new firmware
    echo ">> [2] Compiling new firmware..."

    sleep 0.5

    arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_64:opt=o3std,pnum=NUCLEO_F411RE --verbose srcs/respirator.cpp --output output/respirator-production || exit 1

    # Flash new firmware
    echo ">> [3] Flashing new firmware..."

    sleep 0.5

    arduino-cli upload --port "$SERIAL_PORT" --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_F411RE,upload_method=swdMethod --input output/respirator-production || exit 1

    sleep 0.5

    echo "Success: All done. The ventilator will boot using the new firmware now."
  else
    echo "Error: STM32 programmer device not detected!"

    exit 1
  fi
popd > /dev/null

exit 0
