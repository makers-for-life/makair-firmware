# Changelog

## unreleased

- drop support of electronic hardware v1 and v2
  (_from now on, minimum compatibility is from v3_)
- refactor main loop and breathing control
  (_breathing-related computations are now triggered by a hardware timer_)
- support several breathing modes
  (_PC-CMV stays the default mode, PC-VSAI and PC-AC were added_)
- added volume controlled ventilation modes
  (_VC-CMV and VC-AC were added_)
- telemetry protocol v2 was introduced
  (_basically, this means sending more data_)
- drop support of useless "qualification" and "integration test" modes
  (_both are superseded by the EOL test feature_)
- add a watchdog for the Raspberry Pi
  (_RPi power supply will be restarted if UI misses sending heartbeats_)
- allow to set any setting (current or added) using the serial control protocol
  (_new settings were added, and they can all be changed from UI software_)
- improve breathing control algorithms
- remove ability to set peak pressure
- allow to snooze alarms or start/stop breathing using the serial control protocol
- drop support of old valves and pneumatic systems
- make mass flow meter more reliable
- support for SFM mass flow meters
  (_SFM3019 and SFM_3300D_)
- added support for a mass flow meter on the expiratory branch
  (_if it is missing, the expiratory flow rate will be estimated using an algorithm_)
- turn the green LED on when alarms are snoozed
- added a facility to pre-configure the ventilator settings upon boot, using a patient height setting
  (_submitting the patient height will update all ventilation hyperparameters to optimum values_)
- added configurable alarms for pressure and volume modes, adding to all existing alarms
- sending more measured values for display, eg. respiratory durations and cycles per minute
- a more precise battery voltage value is now sent over the telemetry protocol
  (_this is used for battery SoC estimations_)
- display a special message on LCD screen when watchdog reset was triggered
- ability to detect in a reliable way when AC gets off, and that power is switched to battery
- allow to retry pressure calibration when it fails
- added a ramp-up algorithm on the blower speed controller, in order to prevent current surges in the transformer, which could trigger a temporary handover on batteries
  (_this is a security feature, in cases where batteries would be depleted or not connected, the ventilator would suddently reboot during use_)
- added a compilation flag to disable all hardware buttons
  (_this is used for touchscreen-only MakAir devices, ie. non-traditional ones_)
- added basic support for the motherboard V3 EEPROM chip over I2C
- a lot of potential exceptions due to possible divisions by zero have been fixed
- increase the size of the hardware buffer used by telemetry
- improve Doxygen documentation

## v3.0.1

- fix an issue with the optional mass flow meter

## v3.0.0

- implement a respiratory trigger
  (_this helps the MakAir to respect the patient respiratory reflexes when he/she starts to wake up; disabled by default_)
- add an "end of production line" test program
  (_it is included in the production software and runs when booting while pressing a maintainance button; electronic hardware v2+ required_)
- support Faulhaber motors for valves
  (_they are better, faster, stronger; electronic hardware v2+ required_)
- support an optional mass flow meter
  (_this allows to estimate the volume of inspired air; electronic hardware v2+ required_)
- implement a control protocol to update settings through serial communication
  (_UI on Raspberry Pi can now send new settings values; few more settings are supported compared to physical buttons; electronic hardware v2+ required_)
- support electronic hardware v3
- improve pressure control
- improve blower speed regulation
- fix systick overflow in telemetry protocol
- update telemetry protocol to send more information
- include CRC in telemetry messages
- add a safety to shutdown system if battery is very low
  (_in this situation, everything might get damaged if not shutdown_)
- change some default settings
  (_now, Ppeak starts at 250 mmH2O and Pplateau at 220 mmH2O_)

## v1.5.4

- improve dynamic update of the peak pressure command according to the measured plateau pressure
- fix telemetry bugs
- fix minor bugs

## v1.5.3

- warn if pressure is not stable enough at startup
- improve pressure control
- fix a regression
  (_the PPeak+ button was not functioning anymore_)

## v1.5.2

- improve pressure control

## v1.5.1

- calibrate pressure sensor's offset on startup
- better round displayed pressure values (in cmH2O)
- minor improvement to the pressure control

## v1.5.0

- rework pressure and blower control
  (_blower will now take more time to ramp up/down but this will greatly improve the stability of injected air volume in many scenario_)
- rework alarms
  (_better pressure alarms, better battery alarms, better snooze behavior_)
- improve even more the measured and displayed pressures
  (_if no plateau pressure if found, screen will now display a `?` instead of an uncertain value_)
- tweak the pressure control to make the plateau more accurate
- fix an issue with blower not restarting in some cases
- make sysclock more accurate
- improve code quality (MISRA)
- add a step in integration test to check O2 pipe

## v1.4.0

_This release was depublished_

## v1.3.2

- fix blower control
  (_it used to unexpectedly slow down a bit from times to times_)
- improve the measured and displayed pressures for peak and plateau

## v1.3.1

- use the green LED near the start button to show whether the breathing mode is ON or not
- disable alarms related to the breathing cycle when program is stopped
- fix an issue with a battery alarm being briefly triggered at every boot
- integration test: open both valves at startup

## v1.3.0

- add a program to test integration
- support electronic hardware v1 **and** v2
  (_v1 by default, v2 through a config flag_)
- implement start/stop
  (_from now on, machine will begin stopped!_)
- handle Emerson valves
  (_through a config flag_)
- improve the way buzzer is controlled

## v1.2.3

- fix an issue with buzzer (sometimes it was stuck buzzing whereas no alarm were triggered)
- add battery-related alarms

## v1.2.2

- handle several pneumatic systems
  _(for the first 2 working typologies of prototype)_

## v1.2.1

_Unreleased_

## v1.2.0

First release version
