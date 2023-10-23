---
layout: default
title: Linear actuators
---
# Linear actuators
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---
## Important notes

- ⚠ If the actuator does not receive a command within **1 second**, if will turn off the clutch and enter safe mode. Suggested refresh rate is 100 ms.
- The datasheet notes that for proper care, the clutch should be enabled *at least 20ms before* the motor is enabled, and the motor should be disabled *at least 20ms before disabling the clutch*.

## Basic specs

| Item                      | Value                         |
| ------------------------- | ----------------------------- |
| Input voltage             | 10-30VDC                      |
| Max current               | 5.8A                          |
| Operating temperature     | -40$\degree$C to 85$\degree$C |
| Actuator working distance | 3.1 inches                    |
| CAN baud rate             | 250K (default) or 500K        |



## General message format

Uses 29-bit "extended" CAN2.0b IDs.

Messages are 8 bytes.

| Byte | Description                                        |
| ---- | -------------------------------------------------- |
| 0    | Message data type                                  |
| 1    | Flags, confirmation requests, and data type extras |
| 2-7  | Message data                                       |

### Byte 1

| Bit  | Description                                                  |
| ---- | ------------------------------------------------------------ |
| 7    | Confirmation flag. 1 = Actuator will echo the message back.  |
| 6    | Auto reply flag. 1 = Actuator will reply that it is set up for that message. Does not respond to all messages, only supported ones. |
| 0-5  | Data type                                                    |

## Commands

### `0x0F`: Position command

| Byte.Bit | Description/value                                         |
| -------- | --------------------------------------------------------- |
| 0        | 15 (`0x0f`) *Datatype*                                    |
| 1        | See above. Bits 0-5 must be `0x0A`.                       |
| 2        | `DPOS_LOW`: Least significant byte of DPOS. See below.    |
| 3.7      | Clutch enable. See note below.                            |
| 3.6      | Motor enable.                                             |
| 3.5      | `DPOS_HI`. The most significant 5 bits of the DPOS value. |
| 4-7      | 0 (`0x00`). Not used.                                     |

#### Desired position (`DPOS`)

This is a 13-bit value, the concatenation of `DPOS_HI`+`DPOS_LOW`.

The actual position of the LA is $pos=0.001x+0.5$ inches, where $x$ is `DPOS_LOW`.

Example: `DPOS_HI = 1 0100`, `DPOS_LOW=1000 0100`. Then `DPOS=1 0100 1000 0100`, or 5252 in decimal. Then $pos=0.001\times 5252+0.5=5.752$ inches.

#### Delay in motor enable

The datasheet notes that for proper care, the clutch should be enabled *at least 20ms before* the motor is enabled, and the motor should be disabled *at least 20ms before disabling the clutch*.

### `0xF5_0x00`: Configure baud rate

The baud rate can be set to either 250K (default) or 500K. The specified value is persisted across boots.

| Byte | Description/value                              |
| ---- | ---------------------------------------------- |
| 0    | `0xF5`                                         |
| 1    | See [above](#byte-1). Bits 0-5 must be `0x00`. |
| 2    | `0x00`                                         |
| 3    | `0x00`                                         |
| 4    | `0x00`=250K. `0x01`=500K.                      |
| 5-7  | 0 (`0x00`). Not used.                          |

#### Example

`0xF5 0x00 0x00 0x00 0x01 0x00 0x00 0x00`

Sets baud rate to 500K.

### `0xF5_0x01`: Configure PWM frequency

| Byte | Description/value                              |
| ---- | ---------------------------------------------- |
| 0    | `0xF5`                                         |
| 1    | See [above](#byte-1). Bits 0-5 must be `0x01`. |
| 2    | `0x00`                                         |
| 3    | `0x02`                                         |
| 4    | MIN_PWM, from 0-100%.                          |
| 5    | MAX_PWM, from 0-100%.                          |
| 6-7  | PWM_FREQ, with byte 6 being LSB.               |

These values determine how much force the LA will use.

#### PWM_MAX

Set `PWM_MAX` to be *as low as possible* while still giving the LA enough force to press the pedal.

#### Example

`0xF5 0x01 0 0x02 0x0A 0x5A 0x90 0x01`

This sets MIN PWM to 10% (0x0A) and MAX PWM to 90 (0x5A), and PWM FREQUENCY to 400Hz (0x0190).

*Many commands were omitted. See [datasheet][1] for all messages.*

## Replies

*Not yet noted. See [datasheet][1] for reply information.*



[1]: https://cometmail.sharepoint.com/:b:/s/Voltron/EdV3SAjzPppNgjT-2w6LjbcBmC-LiPZ4OXdMPZ3GIL3AWw?e=gPse2s	"Datasheet"
