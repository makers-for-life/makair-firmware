# MakAir Firmware

[![Firmware Lint](https://github.com/makers-for-life/makair-firmware/workflows/Firmware%20Lint/badge.svg)](https://github.com/makers-for-life/makair-firmware/actions?query=workflow%3A%22Firmware+Lint%22) [![Firmware Unit Tests](https://github.com/makers-for-life/makair-firmware/workflows/Firmware%20Unit%20Tests/badge.svg)](https://github.com/makers-for-life/makair-firmware/actions?query=workflow%3A%22Firmware+Unit+Tests%22)

## Versions

| Version | Last Changelog | Ready? |
| ------- | -------------- | ------ |
| V1.2.x | First release version on Hardware V1 | ✅
| V1.3.x | Add Hardware V2 support | ✅
| V1.5.x | Rework ventilation algorithms (pressure control, alarms, code quality) | ✅
| V3.0.x | Support for triggers, add mass flow meter, add Hardware V3 support | ✅
| V4.0.x | Protocol V2 with ventilation modes, drop Hardware V1 & V2 support | ❌

For a full history of all minor versions, as well as details of all changes, see [CHANGELOG.md](CHANGELOG.md).

## ⚛️ Latest Improvements

A bleeding edge firmware version is available on the [dev](https://github.com/makers-for-life/makair-firmware/tree/dev) branch, which contains our latest improvements, ventilation modes and safety fixes.

**⚠️ Note that the `master` branch contains a legacy firmware version, that will not work with the latest version of [makair-control-ui](https://github.com/makers-for-life/makair-firmware/tree/dev), please use the `dev` branch instead.**

## Releases

Pre-compiled binaries are available in the [Releases](https://github.com/makers-for-life/makair-firmware/releases) section.

## Documentation

Code documentation can be [found there](https://makers-for-life.github.io/makair-firmware/files.html).

## How To Build?

In order to setup your environment and build the code, please follow the following commands (for MacOS):

1. `brew install arduino-cli`
2. `arduino-cli config init --additional-urls https://github.com/stm32duino/BoardManagerFiles/raw/master/package_stmicroelectronics_index.json`
3. `arduino-cli core update-index`
4. `arduino-cli core install STMicroelectronics:stm32@2.0.0`
5. `arduino-cli lib install LiquidCrystal@1.0.7 && arduino-cli lib install OneButton@1.5.0 && arduino-cli lib install CRC32@2.0.0`

Then, compile the project:

```sh
arduino-cli compile --fqbn STMicroelectronics:stm32:Nucleo_64:opt=o3std,pnum=NUCLEO_F411RE --verbose srcs/respirator.cpp --output output/respirator-production
```

## How To Flash?

In order to flash the firmware that you just built using the instructions above:

1. Plug your ST-Link programmer to the motherboard, and switch the motherboard to flash mode (switch position to flash mode on the motherboard);
2. Plug the ST-Link programmer to your computer, over USB, and fetch its device name via the `arduino-cli board list` command (we will call it `{SERIAL_PORT}`);
3. Make sure that [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) is installed on your computer, and run:

```sh
arduino-cli upload --port {SERIAL_PORT} --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_F411RE,upload_method=swdMethod --input output/respirator-production
```

_Make sure to replace {SERIAL_PORT} with your serial port, which should begin with `/dev/`._

## Configuration

High-level configuration options are available and documented in [includes/config.h](includes/config.h).

Low-level configuration options can be found in [includes/parameters.h](includes/parameters.h).
