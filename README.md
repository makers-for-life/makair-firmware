# MakAir Firmware

## Versions

See [CHANGELOG.md](CHANGELOG.md).

Pre-compiled binaries are available in the [Releases](https://github.com/makers-for-life/makair-firmware/releases) section.

## Documentation

Code documentation can be [found there](https://makers-for-life.github.io/makair-firmware/files.html).

## How To Build?

In order to setup your environment and build the code, please follow the following commands (for MacOS):

1. `brew install arduino-cli`
2. `arduino-cli config init --additional-urls https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json`
3. `arduino-cli core update-index`
4. `arduino-cli core install STM32:stm32@1.8.0`
5. `arduino-cli lib install LiquidCrystal@1.0.7 && arduino-cli lib install "Analog Buttons"@1.2.0 && arduino-cli lib install OneButton@1.5.0`

Then, compile the project:

```sh
arduino-cli compile --fqbn STM32:stm32:Nucleo_64:opt=o3std,pnum=NUCLEO_F411RE --verbose srcs/respirator.cpp --output output/respirator-production
```

## Configuration

High-level configuration options are available and documented in [includes/config.h](includes/config.h).

Low-level configuration options can be found in [includes/parameters.h](includes/parameters.h).

## Sonar analysis

The following version of software are build and analysed.

| Software Mode    | Hardware | Valve version | Link                                                                                                  |
| ---------------- | -------- | ------------- | ----------------------------------------------------------------------------------------------------- |
| integration-test | HW1      | pinch valve   | [respirator-integration-test-HW1](https://sonarcloud.io/dashboard?id=respirator-integration-test-HW1) |
| integration-test | HW2      | pinch valve   | [respirator-integration-test-HW2](https://sonarcloud.io/dashboard?id=respirator-integration-test-HW2) |
| production       | HW1      | pinch valve   | [respirator-production-HW1](https://sonarcloud.io/dashboard?id=respirator-production-HW1)             |
| production       | HW2      | pinch valve   | [respirator-production-HW2](https://sonarcloud.io/dashboard?id=respirator-production-HW2)             |
| qualification    | HW1      | pinch valve   | [respirator-qualification-HW1](https://sonarcloud.io/dashboard?id=respirator-qualification-HW1)       |
| qualification    | HW2      | pinch valve   | [respirator-qualification-HW2](https://sonarcloud.io/dashboard?id=respirator-qualification-HW2)       |
