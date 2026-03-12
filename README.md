# WHEELTEC IP570 STM32 Project

STM32 register-level project for the WHEELTEC IP570 linear inverted pendulum.

## Structure

- `USER/`: application logic and controller modules
- `BALANCE/`: balance control, observers, filters, device logic
- `HARDWARE/`: board peripherals and drivers
- `SYSTEM/`: system support code
- `third_party/daqp/`: DAQP quadratic programming solver

## Build

Open `USER/WHEELTEC.uvprojx` with Keil uVision and build the target.

## Notes

- Generated output files under `OBJ/` are ignored by Git.
- Keil user-specific files are ignored by Git.
