# STM32 V1.1 to PL integration notes

The reference firmware maps its first two angle sensors to ADC1 channels 8/9
(PB0/PB1), RP1..RP4 to ADC2 channels 2..5 (PA2..PA5), encoder 1 to PA6/PA7,
encoder 2 to PB6/PB7, keys K1/K2 to PB10/PB11 and K3/K4 to PA11/PA12.  On the
Zynq build these STM32 names are signal roles; they must be assigned to the
actual SmartZynq connector pins in the board XDC.

The original `rtl/xadc4_reader.v` intentionally uses four VAUX channels:
VAUX0 (angle), VAUX2 (RP1), VAUX3 (RP2) and VAUX4 (RP3).  That is suitable for
the earlier three-pot build.  The complete four-pot migration is provided by
`rtl/xadc5_reader.v`: VAUX0=angle and VAUX1..VAUX4=RP1..RP4, with XADC
`INIT_49=16'h001F`.  Use this reader only when all five analog pairs are routed
to the PL and each input has the required 24 kOhm/10 kOhm attenuation to the
XADC's 0..1 V range.

`rtl/key_events.v` is the digital key front end.  Inputs are active-low and
must have a pull-up (the STM32 board uses internal pull-ups).  It synchronizes,
debounces for 20 ms, emits a one-clock `click` on release, and emits one
`long_press` after 1 s.

`rtl/pendulum_ui.v` implements Mode31/Menu behavior.  Its default Q8.8 gains
are Angle Kp=0.25, Ki=0.009, Kd=0.41 and Location Kp=0.52, Ki=0.01, Kd=4.56;
the coefficient windows match the STM32 ranges.  K4 cycles run -> angle-gain
-> location-gain -> menu; K1 starts/stops, K2/K3 move the location target by
408 counts.  The menu limits are center angle 1900..2200, start PWM 0..100,
start time 0..200 ms and PWM offset 0..50.  `USE_RP4_TARGET` defaults to 1 in
this module to support a four-pot position-target build; set it to 0 to retain
the original Mode31 behavior where RP4 is unused.

Use `rtl/angle_center_map.v` for the inner PID measurement when the sensor is
an absolute 0..360 degree potentiometer.  It computes the shortest signed
ADC difference around `center_angle` (default 2050), preserving the STM32
Mode31 convention where the balance target is an ADC count, rather than the
older `angle_from_adc` mapping whose zero is at raw ADC=0.

Before hardware testing, keep TB6612 VM disconnected or use an emergency stop.
Verify STBY is low during reset/startup, then check direction polarity with a
small command.  The angle and encoder signs are mechanical calibration
parameters (`ANGLE_SIGN`, `MOTOR_SIGN`).
