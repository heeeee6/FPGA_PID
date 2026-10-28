# SmartZynq rotary inverted pendulum controller

This Vivado 2018.3 project targets the SmartZynq V1.3B (`xc7z020clg484-1`).
The hardware top is `pid_swing_hw_top`; `tb_pid_swing_top` and
`tb_i2c_txn_master` are behavioral testbenches. The onboard 50 MHz PL clock
uses package pin M19.

## Analog inputs on J5

The angle sensor and four potentiometers feed the XADC through separate
dividers. For each channel, connect the signal through **24 kΩ** to VAUXP,
connect **10 kΩ** from VAUXP to analog ground, and connect its matching VAUXN
to analog ground. A second angle sensor may be fitted on VAUX1; it is left
disabled by default in the hardware wrapper until that sensor is connected.
Share ground between the sensors and FPGA. At a 3.3 V input, the divider gives
about 0.9706 V at VAUXP, corresponding to about 3974 XADC counts. The hardware
wrapper sets `ADC_SPAN_COUNTS=3974`; calibrate this value against measured
hardware before operating the motor.

| Signal | XADC channel | VAUXP package / J5 physical pin | VAUXN package / J5 physical pin |
| --- | --- | --- | --- |
| Pendulum angle | VAUX0 | F16 / 38 | E16 / 37 |
| Optional angle 2 | VAUX1 | E15 / 34 | D15 / 33 |
| RP1 Kp control | VAUX2 | C15 / 30 | B15 / 29 |
| RP2 Ki control | VAUX3 | A16 / 26 | A17 / 25 |
| RP3 Kd control | VAUX4 | D20 / 27 | C20 / 28 |
| RP4 (reserved target knob) | VAUX6 | G19 / 15 | F19 / 16 |

The XADC channel assignments above are verified against the SmartZynq V1.3B
official pin-constraint reference (schematic sheet 11) and the Xilinx
`xc7z020_clg484` package data (`AD0P/N` through `AD6P/N` on bank 35). VAUX5
(`E21`/`D21`) is not brought out on J5, so `vauxp[5]`/`vauxn[5]` are left
unplaced in the XDC.

RP1, RP2 and RP3 map to the angle-loop Kp, Ki and Kd controls on the angle
page. On the location page the same three knobs adjust the outer-loop gains,
matching the STM32 menu. RP4 is read for diagnostics but is disabled as a
position-target input in the default build, so there is no target-speed knob.
Set `USE_SECOND_ANGLE` to `1` in `pid_swing_hw_top.v` only after the optional
second sensor is wired and calibrated.

## Digital wiring on J6

The following is the proposed harness allocation in
`constr/SmartZynq_V1_3B.xdc`. Set **J6 VADJ to 3.3 V** before connecting any
digital device.

| J6 physical pin | Package pin | FPGA signal | Connect to |
| --- | --- | --- | --- |
| 1 | U22 | `motor_pwm` | TB6612 PWMA |
| 2 | T22 | `motor_in1` | TB6612 AIN1 |
| 3 | W22 | `motor_in2` | TB6612 AIN2 |
| 4 | V22 | `motor_stby` | TB6612 STBY |
| 5 | Y21 | `motor_enc_a` | Encoder A, 3.3 V logic |
| 6 | Y20 | `motor_enc_b` | Encoder B, 3.3 V logic |
| 7 | AB22 | `oled_scl` | SSD1306 SCL |
| 8 | AA22 | `oled_sda` | SSD1306 SDA |
| 9 | AB21 | `rst_n` | 3.3 V switch/jumper to release reset |
| 10 | AA21 | `run_enable` | 3.3 V switch/jumper to enable motor |

The J6 pin-to-package mapping above is verified against the SmartZynq V1.3B
official `J6[*]` constraints (schematic sheet 11, bank 33 / VADJ). J6 pins
13-16 (`AA19`, `Y19`, `AB16`, `AA16`) carry the four active-low buttons
K1-K4, matching the STM32 `Key.c` assignment.

## STM32 minimum-system socket compatibility

The PID baseboard drives the motor, OLED, buttons, sensors, and knobs through
the STM32F103C8T6 minimum-system board socket. To keep the FPGA port
pin-compatible, connect each SmartZynq J6 signal to the matching baseboard net
below. The socket pin numbers come from the PID board schematic
(`PID电机驱动及控制板-V1.0.pdf`); the net names and their roles match the
STM32 firmware in `16-倒立摆-自动启摆`.

Digital signals:

| FPGA signal | J6 pin | PID board net | STM32 socket pin |
| --- | --- | --- | --- |
| `motor_pwm` | 1 | TB6612 PWMA | 25 (PA0) |
| `motor_in1` | 2 | TB6612 AIN1 | 1 (PB12) |
| `motor_in2` | 3 | TB6612 AIN2 | 2 (PB13) |
| `motor_stby` | 4 | TB6612 STBY | 3 (PB14), board ties high |
| `motor_enc_a` | 5 | Encoder E1A | 31 (PA6) |
| `motor_enc_b` | 6 | Encoder E1B | 32 (PA7) |
| `oled_scl` | 7 | OLED SCL | 16 (PB8) |
| `oled_sda` | 8 | OLED SDA | 17 (PB9) |
| `key_n[0]` (K1) | 13 | K1 | 35 (PB10) |
| `key_n[1]` (K2) | 14 | K2 | 36 (PB11) |
| `key_n[2]` (K3) | 15 | K3 | 8 (PA11) |
| `key_n[3]` (K4) | 16 | K4 | 9 (PA12) |

Analog signals (0-1 V into XADC after the 24 kΩ / 10 kΩ divider):

| XADC input | J5 pins | PID board net | STM32 socket pin |
| --- | --- | --- | --- |
| VAUX0 (F16/E16) | 38/37 | SENSOR1 (pendulum angle) | 33 (PB0) |
| VAUX1 (E15/D15) | 34/33 | SENSOR2 (optional angle 2) | 34 (PB1) |
| VAUX2 (C15/B15) | 30/29 | RP1 (Kp) | 27 (PA2) |
| VAUX3 (A16/A17) | 26/25 | RP2 (Ki) | 28 (PA3) |
| VAUX4 (D20/C20) | 27/28 | RP3 (Kd) | 29 (PA4) |
| VAUX6 (G19/F19) | 15/16 | RP4 (reserved) | 30 (PA5) |

`rst_n` (J6 pin 9) and `run_enable` (J6 pin 10) are FPGA-only hard-inhibit /
motor-enable inputs; they have no STM32 equivalent and can be wired to any
convenient switch or tied to 3.3 V. The STM32 project leaves TB6612 STBY
enabled at all times, so `motor_stby` is optional if the baseboard already
ties it high; keep the J6 `motor_stby` pulldown in the XDC when it is
connected.

The hardware wrapper uses a 100 ms internal power-on reset. K1 starts and
stops the controller exactly as in the STM32 Mode31 program; after the
3-second commissioning hold the run state machine performs the Mode31
resonant swing-up (40 ms angle sampling with left/right peak detection), then
hands over to the cascade PID when the pendulum enters the capture window and
stops when it actually falls. The motor command ramps by at most one
percentage point per 1 kHz update and is limited to 35%. `rst_n` remains a
hard motor inhibit and has a weak pull-up in the XDC. `run_enable` is always
honored as a physical motor-enable input; pulling J6 pin 10 low returns the
controller to idle and restarts the 3-second hold when released. The
controller and PID state are cleared whenever reset, run disable, startup
hold, or fault is active.
The OLED uses I²C address `0x3C`; SCL
and SDA need pull-ups to **3.3 V**. The hardware wrapper provides open-drain
I/O buffers. Supply the TB6612 with **VM = 12 V** for the motor and
**VCC = 3.3 V** for logic. Connect motor power, FPGA, OLED, encoder, and sensor
grounds together. The motor connects only to the TB6612 output. The TB6612
has no fault output, so the wrapper ties the internal `motor_fault` input low.
Provide a physical motor power stop for commissioning.

## Vivado and simulation

From the project directory, with Vivado 2018.3 on `PATH`:

```powershell
vivado -mode batch -source scripts/update_project.tcl
./scripts/run_sim.ps1
vivado -mode batch -source scripts/check_impl_route.tcl
vivado -mode batch -source scripts/write_bitstream.tcl
```

`update_project.tcl` registers the RTL, both testbenches, and the XDC in
`PID_swing.xpr`, then selects `pid_swing_hw_top` for synthesis and
`tb_pid_swing_top` for the project simulator. `run_sim.ps1` compiles all RTL
and runs every `sim/tb_*.sv` testbench with Vivado XSim. Simulation output is
written under `.sim_check`.

`check_impl_route.tcl` performs synthesis, placement, routing, DRC, and timing
analysis and writes `route_check.dcp`, `route_drc.rpt`, and `route_timing.rpt`.
`write_bitstream.tcl` converts that routed checkpoint into
`pid_swing_hw_top.bit`. On the current Vivado 2018.3 run, all four XSim
testbenches pass (including every 12-bit Kp/Ki mapping input). The routed
50 MHz design has zero DRC errors and hold WHS +0.10 ns. All control-loop,
encoder, XADC, key, and I2C paths meet timing; the single worst setup path is
the OLED velocity text formatter at WNS -0.18 ns, a cosmetic display path.

The RTL and wiring have not been tested on a physical board. During the first
test, keep the 12 V motor supply disconnected or use a physical emergency stop,
confirm that STBY stays low throughout the 3-second hold, and verify the
direction and angle polarity at low command before free-running the pendulum.

The repository also contains `rtl/key_events.v`.  It accepts the four
active-low STM32-compatible key inputs and produces one-clock `click` and
`long_press` events after a 20 ms debounce.  A 1 s hold produces one
`long_press` event and no click on release, matching `Hardware/Key.c`.

`rtl/pendulum_ui.v` contains the Mode31/Menu state machine and parameter
handover logic.  `rtl/angle_center_map.v` converts the absolute angle ADC
sample into a shortest signed error around the configurable STM32 center
angle (default 2050 counts).  These two modules are intentionally separate
from the motor top level so that the pin-compatible wrapper can select the
desired OLED page and control-loop implementation without duplicating key or
angle arithmetic.
