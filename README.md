# BIOBUZZ robot code

Fork of FIRST’s official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) SDK **12.0** (BIOBUZZ 2026–2027), plus this team’s OpModes and **Pedro Pathing 3.0.2** with **Ivy 1.1.1**.

All team Java lives in `TeamCode/`. Do not edit `FtcRobotController/` except when upgrading the SDK.

This file is the only team guide. Official references:

- FTC: https://ftc-docs.firstinspires.org
- Pedro: https://pedropathing.com
- Pedro Quickstart: https://github.com/Pedro-Pathing/Quickstart

---

## 1. Hardware (Robot Controller config)

Use these names exactly. They are stored in `RobotConstants`.

| Device | Type | Config name |
|---|---|---|
| Front left drive | Motor | `left_front` |
| Front right drive | Motor | `right_front` |
| Back left drive | Motor | `left_back` |
| Back right drive | Motor | `right_back` |
| Intake | Motor | `intake` |
| Transfer | Motor | `transfer` |
| Left outtake | Motor | `outtake_left` |
| Right outtake | Motor | `outtake_right` |
| Outtake gate | Servo | `outtake_gate` |
| Turret left | CR Servo | `turretServoLeft` |
| Turret right | CR Servo | `turretServoRight` |
| Turret encoder | REV Through Bore Encoder V2, cable in the **encoder socket of the `intake` motor port** | none (read through `intake`, see `Turret.ENCODER_PORT`) |
| Flower servo | Servo | `flower` |
| goBILDA Pinpoint | I2C | `pinpoint` |

Pinpoint wiring (from Pedro docs):

- Plug into an **I2C port other than 0** (port 0 is the Control Hub IMU)
- Sticker / ports face **up**
- Forward pod → Pinpoint **X**, strafe pod → Pinpoint **Y**

---

## 2. First-time setup

1. Install [Android Studio](https://developer.android.com/studio) (Narwhal 3 or newer).
2. Open **this folder** (the one with `settings.gradle`). Let Gradle sync.
3. Create the configuration on the Robot Controller with the names above.
4. Connect the Robot Controller and Run the `TeamCode` configuration.

Terminal build (Java 17):

```bash
export JAVA_HOME="/Users/deepakpattapu/Library/Java/JavaVirtualMachines/ms-17.0.17/Contents/Home"
./gradlew :TeamCode:assembleDebug
```

You will mostly edit Java under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. You do not need to know all of Android.

---

## 3. Files

Team code is one package. The only extra folder is `pedroPathing/`, which holds Pedro’s official tuner files.

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── RobotConstants.java          Hardware names, powers, PIDF gains, field targets
├── MainTeleOp.java              Match drive + intake/transfer/outtake/servo/turret
├── MainAuto.java                Autonomous from the (56, 8) start (Pedro + Ivy)
├── SecondAuto.java              Autonomous from the (56, 136) start: shoot, then path
├── PoseStorage.java             Hands the last auto / TeleOp pose and turret zero to TeleOp
├── IntakeSubsystem.java         Intake + transfer motors
├── OuttakeSubsystem.java        Dual velocity-controlled outtake motors
├── TurretSubsystem.java         Two-servo turret aimed from the Pedro pose
├── IntakeTest.java              Intake + transfer test
├── OuttakeTest.java             Outtake RPM test
├── MotorPidfFinder.java         Change a motor's velocity PIDF live from the gamepad
├── DrivetrainDirectionTest.java Spin one wheel at a time
├── ToggleButton.java            Press-once helper
└── pedroPathing/                Constants, Tuning, AutoTune procedures
```

| File | Open this when… |
|---|---|
| `RobotConstants.java` | You need a hardware name or a power number |
| `MainTeleOp.java` | You want to change driver controls |
| `IntakeSubsystem.java` | You want to change intake behavior |
| `pedroPathing/Constants.java` | You finished an AutoTune and have numbers to paste |

SDK samples stay in `FtcRobotController/.../external/samples/` and are already `@Disabled`.

---

## 4. Driver Station OpModes

| Name | Group | What it does |
|---|---|---|
| **AutoPath Bottom** | Autonomous | `MainAuto`: starts at (56, 8, 90°) and drives one line. Saves its pose and the turret zero for TeleOp, then the Driver Station preselects the TeleOp named in `OpModeNames.TELEOP_AFTER_AUTO` |
| **AutoPath Top** | Autonomous | `SecondAuto`: starts at (56, 136, 270°), shoots, then drives. Same hand-off to TeleOp |
| **Main TeleOp** | TeleOp | Match drive + intake, transfer, outtake, servo, turret |
| **Test: Intake** | Test | Intake and transfer motors |
| **Test: Outtake** | Test | Dual outtake motors at `Outtake.TARGET_RPM` |
| **Test: Motor PIDF Finder** | Test | Live velocity PIDF tuner for the shooter motors |
| **Test: Drive Directions** | Test | Spin one wheel at a time (A/B/X/Y) |
| **Test: Hardware Bench** | Test | Generic motor/servo bench tester (`motor_0..3`, `servo_1..3`) |
| Pedro AutoTune | Browser | See section 6 (not a normal DS list item) |

Safe order the first time you have a robot: **Test: Drive Directions** → **Test: Intake** → **Main TeleOp** on a stand → **Main TeleOp** on the floor.

Do **not** press gamepad 1 **X** or **B** (drive to a pose) or run Foresight tests until Pinpoint is tuned. Those move the robot using localization.

**Main TeleOp init screen (between Init and Play).** Nothing moves here and the turret servos are unpowered. The screen shows the start pose the turret will aim from; if the robot is not really there, gamepad 1 **Dpad Left** picks the default start pose and zeroes the turret where it points, **Dpad Up** the pose and turret zero from the last autonomous, **Dpad Right** the pose and turret zero from the last TeleOp (for a restart mid-match). It also shows the live turret angle and encoder ticks. Straight after an autonomous the turret zero is kept from that autonomous, so the turret shows its real angle and does not have to be touched; otherwise it must point straight forward and read 0.0 (gamepad 2 **Y** zeroes it where it points). Turning it left by hand must make the angle go up. The turret starts moving by itself on Play.

### Main TeleOp (gamepad 1 — drive)

| Input | Action |
|---|---|
| Left stick | Forward / strafe |
| Right stick X | Turn |
| Dpad Up / Down | Drive speed +10% / -10% |
| Left bumper (hold) | Slow mode (30%) |
| X | Follow a line to the start pose `Field.START_POSE` (cancels if you move a stick) |
| B | Follow a line to the opposite start pose `Field.OPPOSE_POSE` (cancels if you move a stick) |
| Back | Set current heading as field-centric forward (preserves localizer pose for turret). Field-centric drive itself has no button bound right now. |
| A | Toggle servo `flower` between `Servo.POSITION_A` / `POSITION_B` (currently disabled: `FLOWER_SERVO_ENABLED = false` in `MainTeleOp`) |

### Main TeleOp (gamepad 2 — mechanisms)

| Input | Action |
|---|---|
| A (hold) | Intake |
| Y (hold) | Reverse intake |
| X (hold) | Run transfer motor |
| B (hold) | Reverse transfer motor |
| (none, automatic) | Outtake motors run at `Outtake.TARGET_RPM` from Play until Stop |
| Left trigger (hold) | Open outtake gate (closes on release; ignored while RT shoot is running) |
| Right trigger (press) | Run `shootBalls` (wait for flywheels → wait up to `Turret.AIM_WAIT_TIMEOUT_MS` for the turret to be aimed, only while it is tracking → open gate → feed → stop) |
| Dpad Up | Turret: aim at the hive, top or bottom side picked from the robot's Y (`Turret.TOP_TARGET_*` / `BOTTOM_TARGET_*`) |
| Left bumper | Turret: stop tracking and turn 90° from forward; press again to turn back to forward |

The turret is the goBILDA Gear-Driven Turret (2.75:1 Ratio, 105mm ID) driven by two continuous-rotation servos and measured by a REV Through Bore Encoder V2 mounted on one servo's output shaft. Both servos are mounted on the same side, facing the same way, so their direction flags must always match. In Main TeleOp the turret aims continuously from `follower.pose()` every loop, so it tracks while the chassis moves, and picks the top or bottom hive side from the robot's Y position (`Turret.SIDE_SWITCH_Y_INCHES` with a small hysteresis band). All eight motor ports are in use, so the encoder has no port of its own: its quadrature cable plugs into the encoder socket of the `intake` motor port (that motor does not use its own encoder) and the code reads the count through that motor (`Turret.ENCODER_PORT`). If the cable is in a different motor's encoder socket, put that motor's config name there; never use the transfer or outtake ports. The encoder is relative, so "straight forward" has to be recorded once. In a match that happens in autonomous: point the turret straight forward before pressing Play on the autonomous, and its encoder count at that moment is saved for TeleOp, which keeps that zero even if the turret moved during autonomous. If TeleOp is started on its own, the turret is zeroed at Init instead: it must point straight forward then, or press gamepad 2 Y on the init screen once it does. It is never re-zeroed on Play, so a turret nudged after Init is still measured correctly. `Turret.ENCODER_TO_TURRET_RATIO` converts servo-shaft turns to turret turns (1 / 2.75, so one full turret turn is 22528 ticks), and `Turret.ENCODER_REVERSED` flips the sign if turning the turret counterclockwise makes `Turret measuredAngleDeg` go down. Autonomous does not drive the turret; it only records the zero. The pose the turret aims from comes from the last autonomous (both autos save it) or the default start pose, and is shown on the init screen.

**After autonomous.** Both autos are annotated with `preselectTeleOp`, so when an autonomous ends the Driver Station switches to the TeleOp by itself and a driver only has to press Init and Play (code cannot press those). Check this once on your own Driver Station: run an autonomous, let it end, and confirm the TeleOp is selected. Which TeleOp it switches to is one line: `RobotConstants.OpModeNames.TELEOP_AFTER_AUTO`. Every OpMode needs its own name for this to work, which is why the two autos are called "AutoPath Bottom" and "AutoPath Top".

Stick signs: `forward = -left_y`, `strafe = -left_x`, `turn = -right_x`.

Pedro owns the drive motors. TeleOp calls `Constants.create(hardwareMap)`, then `follower.manual(...)` and `follower.update()` every loop. We do not write our own mecanum math.

---

## 5. Pedro in one paragraph

Pedro is a library that knows where the robot is (Pinpoint) and can drive from sticks or follow paths. Create a follower with `Constants.create(hardwareMap)`. Call `follower.update()` every loop. Pose **x/y are inches**, **heading is radians**. On the field, **+X is forward** and **+Y is left**.

This is Pedro **3.0.2** (`com.pedropathing:revhub:3.0.2` + `com.pedropathing:tuning:1.0.0` + `com.pedropathing.ivy:pedro:1.1.1`). It is not the old 2.x `FollowerBuilder` API.

---

## 6. Tuning checklist (do this before match autos)

AutoTune is a webpage on the Robot Controller. Procedures are registered in `pedroPathing/Tuning.java`.

On a laptop connected to Robot Controller Wi-Fi, open:

**http://192.168.43.1:10158**

Then run the tuners **in this order**. After each one, paste the generated Java into `pedroPathing/Constants.java`. Do not invent offsets or Foresight numbers.

### Before you start

- [ ] Config names match the table in section 1
- [ ] Pinpoint on I2C **≠ port 0**, sticker up, pods in X/Y correctly
- [ ] Battery charged; room to push and drive the robot

### 1. Mecanum directions

Docs: https://pedropathing.com/docs/pathing/tuning/drivetrain/mecanum

- [ ] Run **Mecanum Tuner**
- [ ] Enter `left_front`, `right_front`, `left_back`, `right_back`
- [ ] For each wheel, mark forward or reversed
- [ ] Paste `MecanumConfig` into `Constants.java`

### 2. Pinpoint localization

Docs: https://pedropathing.com/docs/pathing/tuning/localization/pinpoint

- [ ] Run **Pinpoint Tuner**
- [ ] Name: `pinpoint` · pick Four Bar, Swing Arm, or Custom
- [ ] Push **forward** when asked, then **left** (+Y is left)
- [ ] Spin ~180° counterclockwise in place for offsets
- [ ] Paste `PinpointConfig` into `Constants.java`
- [ ] **Tests → Pose** or **Localization**: walk the robot; numbers should match the field

Zeros in the repo are placeholders only.

### 3. Foresight (path following)

Docs: https://pedropathing.com/docs/pathing/tuning/foresight

- [ ] Localization looks sane
- [ ] Clear, flat space — the robot drives itself
- [ ] Run **Foresight Tuner**
- [ ] Paste `ForesightConfig` over the starter `0.3` / `0.1` gains

### 4. Prove it, then match-ready leftovers

- [ ] **Tests → Hold** (resists being pushed)
- [ ] **Tests → Line** (forward and back on tape)
- [ ] **Tests → Curve** if you will use curves
- [ ] `Main TeleOp`: robot-centric drive, slow mode, and the start pose on the init screen is right
- [ ] Measure a real start pose (inches + heading)
- [ ] Replace `RobotConstants.Field.START_POSE` / `OPPOSE_POSE` with measured poses before using gamepad 1 **X** / **B**
- [ ] Confirm intake direction on real game pieces

---

## 7. Adding code

Put new OpModes and mechanisms next to `MainTeleOp.java`. Add hardware names to `RobotConstants`. Pedro numbers go in `pedroPathing/Constants.java`.

Never hardcode `"left_front"` or `0.8` in an OpMode.

### Naming

| Thing | Style | Example |
|---|---|---|
| Class | PascalCase | `IntakeSubsystem` |
| Method | camelCase | `toggleIntake()` |
| Constant | UPPER_SNAKE | `INTAKE_POWER` |
| OpMode `name` | Short DS label | `"Main TeleOp"` |
| OpMode `group` | `RobotConstants.OpModeGroups` | `TELEOP`, `TEST`, `AUTO` |

### Subsystems

- Constructor takes `HardwareMap`
- Device name comes from `RobotConstants`
- Public methods say intent (`intake()`, `stop()`), not raw `setPower`
- `update()` writes hardware each loop if needed
- `stop()` is always safe to call from OpMode `stop()`
- Do not `hardwareMap.get` the drive motors in TeleOp — Pedro already owns them

### Comments

1. Class Javadoc: what it is, how it fits the robot, hardware it assumes
2. Public method Javadoc: purpose, parameters, units, side effects
3. Inline comments explain *why* (signs, deadzones, hardware quirks)
4. Section headers in long `loop()` methods
5. `// TODO(team): ...` on anything a human must measure

---

## License

SDK code stays under FIRST’s license in `LICENSE`. Team files in `TeamCode/` are for this team’s robot.
