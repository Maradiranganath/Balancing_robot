# Design Document — Self-Balancing Robot

## 1. System Overview

The robot uses the MPU6050's onboard DMP to compute pitch angle in hardware, which is read by the ESP32-S3 over I2C via an interrupt. A PID control loop converts the angle error into a motor command, driving two BO motors through an L298N dual H-bridge to keep the robot balanced.

```
MPU6050 (DMP) --I2C/INT--> ESP32-S3 --PID--> L298N --> 2x BO Motors
```

## 2. Mechanical Design (Sunboard Frame)

| Part | Quantity | Dimensions |
|---|---|---|
| Base plate | 2 | 6 x 6.5 inches (square) |
| Pillar support | 2 | 3 x 3 inches |
| Pillar support | 1 | 3 x 6 inches |

**Assembly notes:**
- Both BO motors are mounted to the edges of one base plate using Fevibond adhesive.
- The pillar pieces form the vertical stand that holds the controller breadboard above the base.
- L298N module, battery, and switch are mounted using double-sided tape.
- **Battery placement:** to keep the robot balanced, the battery pack is split — 2 cells mounted on one side of the frame, and the other 2 cells mounted on the opposite side, rather than placing the full pack on one side.

## 3. Electrical Connections

### 3.1 MPU6050 → ESP32-S3

| MPU6050 Pin | ESP32-S3 Pin |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 8 |
| SCL | GPIO 9 |
| INT | GPIO 2 |

### 3.2 L298N → ESP32-S3 (logic side)

| L298N Pin | ESP32-S3 Pin |
|---|---|
| ENA | GPIO 6 |
| IN1 | GPIO 1 |
| IN2 | GPIO 3 |
| IN3 | GPIO 4 |
| IN4 | GPIO 5 |
| ENB | GPIO 7 |
| GND | GND (common with ESP32 GND) |

Also connect L298N's 5V output and GND to the ESP32-S3's VBUS and GND pins (on the breadboard) to power the controller from the module's onboard 5V regulator.

### 3.3 L298N → Motors

| L298N Pin | Motor |
|---|---|
| OUT1 / OUT2 | Motor 1 |
| OUT3 / OUT4 | Motor 2 |

### 3.4 Battery Connection

| Connection | Details |
|---|---|
| Battery+ | Directly to L298N 12V input terminal |
| Battery− | Through the power switch, then to L298N GND |
| Battery type | 2S Li-ion pack (7.4V nominal), physically split 2 cells per side of the frame for balance |

**Note:** the switch is wired on the ground (negative) side of the battery, not the positive side. Both configurations work correctly for this build since the circuit shares one common ground; this is simply the configuration used here.

### 3.5 Power Distribution Summary

```
Battery+ ----------------------------> L298N 12V IN
Battery- --> [Switch] -------------> L298N GND --> ESP32 GND (common)
L298N 5V OUT -----------------------> ESP32 VBUS
L298N GND ---------------------------> ESP32 GND
```

## 4. Firmware Architecture

- **Sensor fusion:** handled entirely by MPU6050's onboard DMP (no manual complementary filter) — orientation data is retrieved as quaternion → yaw/pitch/roll on each DMP interrupt.
- **Control loop:** `PID_v1` library computes `output` from `input` (current pitch) vs `setpoint` (balance target angle).
- **Motor driving:** `LMotorController` library converts PID `output` into differential PWM signals for both motors via the L298N.

## 5. Tuning Parameters

| Parameter | Purpose | Tuned Value (this build) |
|---|---|---|
| `originalSetpoint` | Target "balanced" angle — depends on MPU6050 mounting orientation | 165.0 |
| `Kp` | Proportional gain — reacts to current tilt error | 85 |
| `Ki` | Integral gain — corrects slow persistent lean | 260 |
| `Kd` | Derivative gain — damps oscillation | 2.2 |
| `motorSpeedFactorLeft` | Compensates for left motor speed mismatch | 0.9 |
| `motorSpeedFactorRight` | Compensates for right motor speed mismatch | 0.9 |
| `MIN_ABS_SPEED` | Minimum PWM to overcome motor deadzone | 30 (placeholder — measure per motor) |

### Calibration procedure

1. **Setpoint:** Temporarily disable the PID/motor output lines, print `input` to Serial, hold the robot perfectly upright by hand, and note the value it settles at. Use that as `originalSetpoint`.
2. **PID gains:** Set `Ki = 0`, `Kd = 0`. Increase `Kp` until the robot visibly tries to balance and begins a steady oscillation. Then increase `Kd` to damp that oscillation. Finally, increase `Ki` slightly, only enough to remove any slow persistent lean.
3. **Deadzone (`MIN_ABS_SPEED`):** With wheels off the ground, ramp PWM up from 0 until the motor just starts turning — use that value (plus a small margin) as the deadzone.

## 6. Known Limitations

- `MIN_ABS_SPEED` is a placeholder and should be re-measured for your specific BO motors — values vary motor to motor.
- Gyro/accel offsets (`setXGyroOffset`, etc.) are set to 0 by default — for best accuracy, run the MPU6050 `IMU_Zero` example sketch once and use the measured offsets instead.
