#!/bin/bash

ABSPATH=$(cd "$(dirname "$0")"; pwd)
BASE_DIR="$ABSPATH/../"

pushd "$BASE_DIR" > /dev/null
  # Clear out built code
  echo ">> [1] Clearing old builds..."

  sleep 0.5

  rm -f ./builds/* || exit 1

  echo "Old builds cleared."

  # Compile new firmware
  echo ">> [2] Compiling new firmware..."

  sleep 0.5

  arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_64:opt=o3std,pnum=NUCLEO_F411RE --verbose srcs/respirator.cpp --output builds/respirator-production || exit 1

  sleep 0.5

  echo "Success: All done. Firmware was compiled successfully."
popd > /dev/null

exit 0
