# 🚀 Flight Control Computer

**Aeronist Aerospace Rocket Team** · Embedded software of the flight control computer developed for the **TEKNOFEST 2026 Mid-Altitude Rocket Competition**.

![Platform](https://img.shields.io/badge/MCU-STM32F446RCT6-03234B)
![Language](https://img.shields.io/badge/Language-C-00599C)
![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE-blue)
![Architecture](https://img.shields.io/badge/Architecture-Super%20Loop%20%2B%20IRQ-informational)

It collects sensor data, estimates attitude and vertical velocity, determines the flight phases and triggers the recovery system (drogue + main parachute). All flight data is logged to external flash memory; if a reset occurs during flight, the system resumes where it left off using the data backed up to backup SRAM.

---

## Contents

- [Layered Structure](#layered-structure)
- [Modules](#modules)
- [Main Loop](#main-loop)
- [Flight Algorithm](#flight-algorithm)
- [Operating Modes](#operating-modes)
- [Hardware](#hardware)
- [References](#references)

---

## Layered Structure

All modules communicate through a single shared data block: `FlightData` ([flight_data.h](Core/Inc/flight_data.h)). Four modules touch the block — `data_manager`, `sensor_fusion`, `flight_algorithm` and `backup_domain`; `quaternion` and `kalman_filter` never see the struct, they are only passed floats.

Ownership of the sections is exactly as follows:

| Section | Written by |
| :--- | :--- |
| `raw` | `data_manager` — except `raw.vertical_accel`, which is computed by `sensor_fusion` |
| `filtered` | `sensor_fusion` — in SUT mode, `data_manager` overwrites the `theta` and `angle_*` fields with simulation values |
| `flags` | set by `data_manager`, cleared by `sensor_fusion` |
| `time` | `data_manager`, except `kalman_dt` (measured by `sensor_fusion`) |
| `sut_state_packet` | `flight_algorithm` |
| `bkpsram_packet` | `data_manager` |

The flight algorithm reads only the `filtered` section when making decisions.

<img src="schematics/architecture.svg" alt="UKB layered architecture" width="100%">

`sensor_check` and `theta_check` are not separate stages but plausibility checks: `sensor_check` is run by `KalmanFilter_Update()` on its own two measurements, and `theta_check` is called by `FlightAlgorithm_Update()`. There are two feedback paths from the decision layer to the estimation layer: `theta_check`'s angle criterion flag and the flight phase. When the flag drops, `flight_algorithm` abandons the angle-based apogee criterion and `sensor_fusion` computes vertical acceleration assuming a fixed 10° instead of the estimated tilt angle. Three modules read the flight phase via `flight_algorithm_get_flight_phase()`: `sensor_fusion` computes vertical acceleration with a different formula in `PHASE_IDLE`, `kalman_filter` applies pad freezing only in `PHASE_IDLE`, and `sensor_check` keeps the acceleration jump limit at 150 m/s² in `PHASE_IDLE`/`PHASE_BOOST` and at 10 m/s² after burnout.

---

## Modules

| Module | Layer | Role |
| :--- | :--- | :--- |
| [bmp390.c](Core/Src/bmp390.c) | Driver | Barometer driver, ground reference altitude |
| [bmi088.c](Core/Src/bmi088.c) | Driver | IMU driver, gyro bias calibration |
| [w25q.c](Core/Src/w25q.c) | Driver | Flash driver, empty page search, command interface |
| [uart_handler.c](Core/Src/uart_handler.c) | Driver | SIT/SUT packet parsing and transmission (DMA) |
| [backup_domain.c](Core/Src/backup_domain.c) | Driver | Backup SRAM + RTC, reset detection |
| [data_manager.c](Core/Src/data_manager.c) | Data | Raw data acquisition, axis transformation, packet building |
| [quaternion.c](Core/Src/quaternion.c) | Estimation | Adaptive Mahony + gyro-only attitude estimation |
| [sensor_check.c](Core/Src/sensor_check.c) | Estimation | Sensor health check (called by `KalmanFilter_Update()`) |
| [kalman_filter.c](Core/Src/kalman_filter.c) | Estimation | 1D Kalman, mach transition, pad freezing |
| [sensor_fusion.c](Core/Src/sensor_fusion.c) | Estimation | Orchestration of the quaternion + Kalman calls |
| [theta_check.c](Core/Src/theta_check.c) | Decision | Theta plausibility check (called by `FlightAlgorithm_Update()`) |
| [flight_algorithm.c](Core/Src/flight_algorithm.c) | Decision | Phase state machine, apogee detection, pyro triggering |

---

## Main Loop

Super loop + interrupt-driven data acquisition. Sensor reads are triggered by data-ready EXTI, while the decision logic runs synchronously with TIM2's 100 ms flag (10 Hz).

<img src="schematics/loop.svg" alt="UKB main loop" width="100%">

---

## Flight Algorithm

<img src="schematics/algorithm.svg" alt="UKB flight algorithm" width="100%">

`×N` is the number of consecutive confirmations required for a transition to be accepted. The drogue parachute deploys within `PHASE_COAST` and the phase does not change; the transition to `PHASE_MAIN_DESCENT` is determined by the main parachute command. Arming is one-way and is a prerequisite for **every** parachute command — apogee does not trigger without arming. Whichever of the two apogee criteria confirms first deploys the drogue.

### Thresholds

All constants of the decision logic are in the `#define` block at the top of [flight_algorithm.c](Core/Src/flight_algorithm.c#L14-L34). All measurements are read from the `filtered` section at 10 Hz; a transition is accepted only if the condition holds for **N consecutive samples**.

| Transition / Event | Condition | Confirmation |
| :--- | :--- | :---: |
| Liftoff (`IDLE` → `BOOST`) | vertical velocity > 35 m/s | ×4 |
| Burnout (`BOOST` → `COAST`) | vertical acceleration < 0 m/s² | ×4 |
| Recovery arming | altitude > 2200 m | ×5 |
| Apogee — angle criterion | θ > 70° | ×3 |
| Apogee — velocity criterion | vertical velocity < 2 m/s | ×5 |
| Main parachute (`COAST` → `MAIN_DESCENT`) | altitude < 500 m | ×3 |
| Landing (`MAIN_DESCENT` → `LANDED`) | −0.1 < vertical velocity < 0.1 m/s **or** altitude < 30 m | ×3 |

The pyro channels are automatically released `PARACHUTE_CHANNELS_ACTIVE_TIME` = 1200 ms after being fired.

The angle criterion is active only while the `theta_check` plausibility check is valid; if the check finds θ unreliable, apogee detection falls back to the velocity criterion alone.

---

## Operating Modes

The board runs in three modes. The mode is selected by a 5-byte command packet from the ground station and written to backup SRAM; the mode is preserved across a reset.

| Mode | Command | Sensor source | Flight algorithm | Transmitted |
| :--- | :---: | :--- | :--- | :--- |
| `NORMAL_MODE` | `0x24` | Board's own sensors | Runs | — (no telemetry) |
| `SIT_MODE` | `0x20` | Board's own sensors | Does not run | 36 B SIT telemetry packet |
| `SUT_MODE` | `0x22` | Incoming simulation packet | Runs | 6 B SUT state packet |

**NORMAL** is the flight mode; the board only listens and transmits nothing. **SIT** (Sensor Monitoring Test) streams the board's own sensor data to the ground station; the decision logic is not active. **SUT** (Synthetic Flight Test) replaces the sensor readings with the incoming simulation data and sends back only the algorithm's state — phase transitions, apogee criteria and parachute commands are reported bit by bit in `sut_state_packet` ([data_manager.h](Core/Inc/data_manager.h)).

`FlightAlgorithm_Reset()` and `KalmanFilter_Reset()` are called on every mode command, so each test starts from a clean state. Flight data is logged to flash in all three modes.

Four frame types share the line — command 5 B, SUT data 36 B, SIT telemetry 36 B, SUT state 6 B. Incoming frames are distinguished by their length; each frame ends with a single-byte sum checksum and `0x0D 0x0A`. Reception is fully interrupt-driven via `HAL_UARTEx_ReceiveToIdle_DMA()`; validation is done in the main loop. The frame layout is documented in [uart_handler.h](Core/Inc/uart_handler.h), the payload layouts in [data_manager.h](Core/Inc/data_manager.h).

The active test port is selected with the `UKB_TEST_WITH_TYPE_C` compile-time switch in [main.c](Core/Src/main.c#L48): if defined, USART2 (Type-C); otherwise USART3 (RS232).

---

## Hardware

| Component | Model | Interface |
| :--- | :--- | :--- |
| Main processor | STM32F446RCT6 | 160 MHz (HSE 8 MHz → PLL), RTC LSE 32.768 kHz |
| Barometer | BMP390 | SPI2, data-ready EXTI (falls back to polling on timeout) |
| IMU | BMI088 | SPI2, data-ready EXTI |
| Flash memory | W25Q16 (2 MB) | SPI1, DMA writes |
| Test port | FT232 (USB Type-C) / MAX3232 (RS232) | USART2 / USART3 |
| Recovery | 2 pyro channels + buzzer | GPIO |

The board has no GPS or telemetry module; the communication computer is a separate unit.

Sensor configuration: accelerometer ±24 g / 200 Hz / 20 Hz bandwidth, gyroscope ±2000 °/s / 1000 Hz / 116 Hz, barometer normal mode / 50 Hz / pressure ×4 oversampling / IIR off. The rocket's longitudinal axis is **accel_x**; the transformation `(x, y, z) → (-x, -y, z)` is applied from the sensor frame to the body frame.

Timers: **TIM2** 100 ms periodic interrupt (main loop trigger), **TIM5** 32-bit free-running counter (gyro / accel / Kalman `dt` measurement, 1 µs resolution).

### Pin Map

| Pin | Function |
| :--- | :--- |
| PB13 / PB14 / PB15 | SPI2 SCK / MISO / MOSI (shared sensor bus) |
| PA8 | ACC_CS (accelerometer chip select) |
| PB12 | GYRO_CS (gyroscope chip select) |
| PA12 | BARO_CS (barometer chip select) |
| PC6 | ACC_INT (accelerometer data-ready EXTI) |
| PA9 | GYRO_INT (gyroscope data-ready EXTI) |
| PA11 | BARO_INT (barometer data-ready EXTI) |
| PC9 | SENSOR_EN (sensor power enable) |
| PB3 / PB4 / PB5 | SPI1 SCK / MISO / MOSI (flash bus) |
| PB6 | W25Q_CS (flash chip select) |
| PA2 / PA3 | USART2 TX / RX (FT232 — USB Type-C) |
| PC10 / PC11 | USART3 TX / RX (MAX3232 — RS232) |
| PB10 | RECOVERY_2 (drogue parachute pyro channel) |
| PA5 | RECOVERY_1 (main parachute pyro channel) |
| PA0 | BUZZER |
| PB8 | USER_LED |

---

## References

- https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf
- https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
- https://github.com/yahio61/bmi_088_library/blob/main/bmi088/bmi088.h · https://github.com/yahio61/bmi_088_library/blob/main/bmi088/bmi088.c
- https://docs.rs-online.com/dea4/0900766b81622f8e.pdf
- https://github.com/halilsrky/SkyLord/blob/main/Core/Inc/queternion.h · https://github.com/halilsrky/SkyLord/blob/main/Core/Src/queternion.c
