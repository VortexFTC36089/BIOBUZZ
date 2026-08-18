# BIOBUZZ TeamCode

Team-owned robot software for FTC DECODE (2025–2026). Official SDK code lives in `FtcRobotController/` and should not be edited for robot features.

## Hardware map

These names must match the Driver Hub configuration **exactly**:

| Device | Config name | Type |
|---|---|---|
| Front-left drive | `leftFront` | Motor (DcMotorEx) |
| Front-right drive | `rightFront` | Motor (DcMotorEx) |
| Rear-left drive | `leftRear` | Motor (DcMotorEx) |
| Rear-right drive | `rightRear` | Motor (DcMotorEx) |
| Intake | `intake` | Motor (DcMotorEx) |

If INIT fails with a missing hardware device, fix the Driver Hub names or update `constants/Constants.java`.

## Architecture

```text
OpMode  →  Robot  →  subsystems (Drivetrain, Intake)
                ↘  RobotHardware (only place that calls hardwareMap.get)
```

- `constants/` — hardware names and tunable values
- `hardware/` — hardware initialization
- `subsystems/` — mechanism APIs used by OpModes
- `robot/` — wires hardware to subsystems
- `opmodes/teleop/` — competition TeleOp
- `opmodes/testing/` — isolated hardware checks

## OpModes

| Driver Station name | Group | Purpose |
|---|---|---|
| Main TeleOp | Competition | Drive + intake |
| Drivetrain Test | Testing | One-motor direction / encoder check |
| Intake Test | Testing | Intake / outtake power check |

## First robot bring-up

1. Create a robot configuration on the Driver Hub using the names above.
2. Run **Drivetrain Test**. Hold A: robot should drive straight forward; all encoder counts should increase.
3. If a wheel runs backward, reverse that motor in `RobotHardware`, not in TeleOp.
4. Run **Intake Test**. Confirm collection vs expel directions.
5. Run **Main TeleOp**.

## Adding a subsystem

1. Add the hardware name to `Constants`.
2. Initialize the device in `RobotHardware`.
3. Create a class under `subsystems/`.
4. Construct it in `Robot`.
5. Call it from TeleOp / Auto.
6. Add a testing OpMode.

## What not to do yet

Do not add Road Runner, a command framework, PID wrappers, or vision until TeleOp is reliable on the robot.
