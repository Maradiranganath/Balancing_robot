# Self-Balancing Robot (ESP32-S3 + MPU6050 DMP)

A two-wheeled self-balancing robot built with a low-cost sunboard frame — no 3D-printed or machined parts required. Uses the MPU6050's onboard DMP for orientation sensing and a PID control loop to keep the robot upright.

![Balancing Robot](docs/robot_photo.jpg)

## Features

- Self-balances using MPU6050's hardware DMP (Digital Motion Processor) — no manual sensor fusion code needed
- Built entirely from sunboard — easy to cut, no frame fabrication tools required
- Fully tunable PID parameters

## Hardware Used

| Component | Qty | Notes |
|---|---|---|
| ESP32-S3 dev board | 1 | Main controller |
| MPU6050 module | 1 | IMU, DMP-based orientation |
| 200RPM BO motor | 2 | Drive motors |
| 65mm wheel | 2 | Matched to BO motor shaft |
| L298N motor driver | 1 | Dual H-bridge |
| 2S Li-ion battery pack | 1 | Power source |
| Sunboard sheet | - | Frame material |
| Fevibond adhesive | - | Motor mounting |
| Double-sided tape | - | Component mounting |

See [DESIGN.md](DESIGN.md) for full wiring and mechanical details.

## Software / Libraries Required

This repo includes the exact library versions used for this build — copy them into your Arduino libraries folder rather than installing different versions from the Library Manager, to avoid compatibility issues (see Step 10 below).

- `PID_v1`
- `I2Cdev` + `MPU6050_6Axis_MotionApps20`
- `LMotorController`

## Build Steps

### Mechanical Build

1. Cut two square sunboard pieces, 6 x 6.5 inches each — these form the base.
2. Stick both BO motors onto one base square, at the edges, using Fevibond adhesive.
3. Take the wires out from both BO motors.
4. Cut two pieces measuring 3 x 3 inches, and one piece measuring 3 x 6 inches — these form the pillar/stand frame.
5. Assemble the pillar frame on the base to hold the controller breadboard above it.

### Power Wiring

6. Connect the battery directly to the L298N module's 12V input terminal.
7. Connect the switch on the ground side — between the battery's negative terminal and the module's GND.
8. Connect both motors to the L298N: one motor to OUT1/OUT2, the other to OUT3/OUT4.
9. Mount the L298N module onto the frame using double-sided tape. Mount the battery and switch the same way.
   - To keep the robot balanced, split the battery pack — 2 cells on one side of the frame, 2 cells on the opposite side — rather than placing the full pack on one side.

### Controller Wiring

10. Place the ESP32-S3 onto a mini breadboard — this is the controller.
11. Place the MPU6050 onto the same breadboard.
12. Connect MPU6050 to ESP32-S3: VCC→3.3V, GND→GND, SDA→GPIO8, SCL→GPIO9, INT→GPIO2.
13. Connect jumper wires from the L298N's ENA, IN1, IN2, IN3, IN4, and ENB pins to the ESP32-S3's GPIO6, GPIO1, GPIO3, GPIO4, GPIO5, and GPIO7 respectively.
14. Connect the L298N's 5V and GND outputs to the ESP32-S3's VBUS and GND pins on the breadboard.
15. Build a small stand on the frame and place the breadboard controller circuit onto it.

### Software Setup

16. Copy the library folders included in this repo into your Arduino IDE's `libraries` folder (usually `Documents/Arduino/libraries`).
17. Connect a USB cable from your PC to the ESP32-S3.
18. Open the firmware file (`balancing_robot_dmp.ino`) in Arduino IDE.
19. Calibrate `originalSetpoint`: temporarily disable the PID/motor output lines, print `input` to Serial, hold the robot perfectly upright by hand, and note the value it settles at — use that as your `originalSetpoint`.
20. Adjust `Kp`, `Ki`, `Kd`, and `motorSpeedFactorLeft` / `motorSpeedFactorRight` for your own build (see tuned values below as a starting reference).
21. Upload the code.

## Usage

1. Power on via the battery switch.
2. Place the robot upright and let go — it should begin balancing within a second.
3. Re-adjust `Kp` / `Ki` / `Kd` / `originalSetpoint` if it drifts or oscillates — see tuning notes in DESIGN.md.

## Tuned Values (this build)

```cpp
double originalSetpoint = 165.0;
double Kp = 85;
double Kd = 2.2;
double Ki = 260;
double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;
```

Your own values will differ slightly based on your MPU6050's mounting angle and motor characteristics — see the calibration steps in DESIGN.md.

## Credits

Built by Maradi — Maradi Innovations
Follow the channel for more DIY electronics, robotics, and automation builds.

## License

Add your preferred license here (e.g., MIT).
