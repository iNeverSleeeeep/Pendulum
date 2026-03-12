# WHEELTEC IP570 STM32 Project

STM32 register-level project for the WHEELTEC IP570 linear inverted pendulum.

## Structure

- `USER/`: application logic and controller modules
- `BALANCE/`: balance control, observers, filters, device logic
- `HARDWARE/`: board peripherals and drivers
- `SYSTEM/`: system support code
- `third_party/daqp/`: DAQP quadratic programming solver

## MPC

`USER/module_mpc.c` contains the MPC controller implementation.

- The matrix values `A`, `B`, `Q`, `F`, `R` are currently kept in source.
- The QP solver interface keeps the name `quadprog()`.
- The backend has been adapted to use `DAQP` for embedded solving.

## Build

Open `USER/WHEELTEC.uvprojx` with Keil uVision and build the target.

## Notes

- Generated output files under `OBJ/` are ignored by Git.
- Keil user-specific files are ignored by Git.
