# BIOBUZZ robot code

Fork of FIRST’s official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) SDK **12.0** (BIOBUZZ 2026–2027), plus this team’s OpModes and **Pedro Pathing 3.0.1**.

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
| Front left drive | Motor | `frontLeft` |
| Front right drive | Motor | `frontRight` |
| Back left drive | Motor | `backLeft` |
| Back right drive | Motor | `backRight` |
| Intake | Motor | `intake` |
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

## 3. Folder structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── RobotConstants.java     Hardware names, intake power, slow mode, home pose
├── opmodes/teleop/         MainTeleOp
├── opmodes/auto/           Match autos (empty until paths are written)
├── opmodes/test/           Intake and drivetrain direction tests
├── subsystems/             One class per mechanism (IntakeSubsystem)
├── pedroPathing/           Constants, Tuning, official AutoTune procedures
├── util/                   Small helpers (ToggleButton)
└── samples/                Optional copies of SDK samples (keep @Disabled)
```

| File | Open this when… |
|---|---|
| `RobotConstants.java` | You need a hardware name or a power number |
| `opmodes/teleop/MainTeleOp.java` | You want to change driver controls |
| `subsystems/IntakeSubsystem.java` | You want to change intake behavior |
| `pedroPathing/Constants.java` | You finished an AutoTune and have numbers to paste |

SDK sample OpModes stay in `FtcRobotController/.../external/samples/` and are already `@Disabled`. Copy one into `samples/` only if you want to start from it.

---

## 4. Driver Station OpModes

| Name | Group | What it does |
|---|---|---|
| **Main TeleOp** | TeleOp | Match drive + intake |
| **Test: Intake** | Test | Only the intake motor |
| **Test: Drive Directions** | Test | Spin one wheel at a time (A/B/X/Y) |
| Pedro AutoTune | Browser | See section 6 (not a normal DS list item) |

Safe order the first time you have a robot: **Test: Drive Directions** → **Test: Intake** → **Main TeleOp** on a stand → **Main TeleOp** on the floor.

Do **not** press **Y (drive to home)** or run Foresight tests until Pinpoint is tuned. Those move the robot using localization.

### Main TeleOp (gamepad 1)

| Input | Action |
|---|---|
| Left stick | Forward / strafe |
| Right stick X | Turn |
| Left bumper (hold) | Slow mode (30%) |
| X | Toggle field-centric |
| Back | Reset heading to 0 (keeps x/y) |
| Right trigger | Intake while held |
| Left trigger | Reverse while held |
| A | Latch continuous intake on/off |
| Y | Follow a line to the home pose (cancels if you move a stick) |

Stick signs follow Pedro’s TeleOp guide: `-left_y`, `+left_x`, `+right_x`.

Pedro owns the drive motors. TeleOp calls `Constants.create(hardwareMap)`, then `follower.manual(...)` and `follower.update()` every loop. We do not write our own mecanum math.

---

## 5. Pedro in one paragraph

Pedro is a library that knows where the robot is (Pinpoint) and can drive from sticks or follow paths. Create a follower with `Constants.create(hardwareMap)` (or `createFollower`). Call `follower.update()` every loop. Pose **x/y are inches**, **heading is radians**. On the field, **+X is forward** and **+Y is left**.

This is Pedro **3.0.1** (`com.pedropathing:revhub:3.0.1` + `com.pedropathing:tuning:1.0.1`). It is not the old 2.x `FollowerBuilder` API.

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
- [ ] Enter `frontLeft`, `frontRight`, `backLeft`, `backRight`
- [ ] For each wheel, mark forward or reversed
- [ ] Paste `MecanumConfig` into `Constants.java`
- [ ] Keep `c.manualBrakeMode.set(true)`

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
- [ ] `Main TeleOp`: robot-centric, field-centric, heading reset
- [ ] Measure a real start pose (inches + heading)
- [ ] Replace `RobotConstants.Field.HOME_*` if you use **Y**
- [ ] Confirm intake direction on real game pieces

---

## 7. Adding code

| You want… | Put it here |
|---|---|
| A new driver program | `opmodes/teleop` |
| A match autonomous | `opmodes/auto` |
| A hardware test | `opmodes/test` |
| A new mechanism | `subsystems` + a name in `RobotConstants` |

Never hardcode `"frontLeft"` or `0.8` in an OpMode. Read `RobotConstants` (and `pedroPathing/Constants` for Pedro).

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
