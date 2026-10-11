# PS-02 Register Map
## Mode-Switching Hardware PID Controller

### Overview
The PID controller exposes a 32-bit register interface for configuration by the flight computer (via SPI or memory-mapped bus). All registers are 32-bit aligned.

### Register Map

| Address | Name         | R/W | Bits   | Description                                       |
|---------|--------------|-----|--------|---------------------------------------------------|
| 0x00    | CTRL         | R/W | [1:0]  | Mode select: 0=Acro, 1=Stabilize, 2=AltHold, 3=Failsafe |
|         |              |     | [2]    | Reset (1 = active)                                 |
|         |              |     | [3]    | Enable watchdog                                    |
| 0x04    | STATUS       | R   | [0]    | Saturated flag                                     |
|         |              |     | [1]    | Failsafe triggered                                 |
|         |              |     | [2]    | SPI data valid                                     |
| 0x08    | SETPOINT_ATT | R/W | [15:0] | Attitude setpoint (Q4.12)                          |
| 0x0C    | MEAS_ATT     | R   | [15:0] | Attitude measurement (Q4.12) from IMU              |
| 0x10    | MEAS_RATE    | R   | [15:0] | Rate measurement (Q4.12) from IMU                  |
| 0x14    | PWM_OUT      | R   | [15:0] | Actuator output (Q4.12)                            |
| 0x18    | KP_ATT       | R/W | [15:0] | Attitude loop Kp (Q4.12)                           |
| 0x1C    | KI_ATT       | R/W | [15:0] | Attitude loop Ki (Q4.12)                           |
| 0x20    | KD_ATT       | R/W | [15:0] | Attitude loop Kd (Q4.12)                           |
| 0x24    | KP_RATE      | R/W | [15:0] | Rate loop Kp (Q4.12)                               |
| 0x28    | KI_RATE      | R/W | [15:0] | Rate loop Ki (Q4.12)                               |
| 0x2C    | KD_RATE      | R/W | [15:0] | Rate loop Kd (Q4.12)                               |
| 0x30    | WD_TIMEOUT   | R/W | [31:0] | Watchdog timeout in clock cycles (default 50000)   |

### Gain Banks
Each flight mode loads a different gain set automatically on mode switch:
- Acro:      only rate gains are applied
- Stabilize: attitude gains active, rate gains applied
- AltHold:   stabilize gains + vertical channel gain
- Failsafe:  output forced to 0, gains ignored

### Q4.12 Fixed-Point Format
All numerical values use Q4.12 format:
- 1 sign bit
- 3 integer bits (range -8.0 to +7.999)
- 12 fractional bits (resolution 1/4096 ≈ 0.000244)
- Example: 0x1000 = 1.0, 0x0800 = 0.5, 0xF000 = -1.0

### SPI Interface (Suggested)
- 4-wire SPI: SCLK, MOSI, MISO, CS
- Address phase: 1 byte (register offset)
- Data phase: 4 bytes (32-bit value, MSB first)
- Max SCLK: 25 MHz
- Typical IMU update rate: 1-8 kHz
