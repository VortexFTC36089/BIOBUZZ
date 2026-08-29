# BIOBUZZ robot code (first year)

This folder is the only place you should write robot code.

Do **not** edit files inside `FtcRobotController`. Those belong to FIRST.

## Which file do I open?

| File | Open it when you want to... |
|---|---|
| `Robot.java` | Change motor names, directions, or speeds |
| `TeleOpMain.java` | Change driver buttons |
| `DriveTest.java` | Check that each wheel works |
| `IntakeTest.java` | Check that the intake works |
| `Auto.java` | Change what the robot does by itself at the start of a match |

On the Driver Station those programs show up as:

- **TeleOp**
- **Drive Test**
- **Intake Test**
- **Simple Auto**

## Motor names

These names must match the robot configuration on the Driver Hub **exactly** (spelling and capital letters):

- `leftFront`
- `rightFront`
- `leftRear`
- `rightRear`
- `intake`

If the app crashes when you press INIT and says it cannot find a device, the name on the Hub does not match `Robot.java`.

## First time on the robot

1. Make a robot configuration on the Driver Hub using the names above.
2. Run **Drive Test**. Hold **A**. The robot should drive straight forward.
3. If one wheel goes the wrong way, open `Robot.java` and swap `FORWARD` / `REVERSE` on that motor.
4. Run **Intake Test**. Right trigger should pull pieces in.
5. Run **TeleOp**.

## Common problems

**Robot does nothing / crashes on INIT**  
A motor name is wrong. Check the Driver Hub config against `Robot.java`.

**Robot drives backward when I push the stick forward**  
Swap `FORWARD` and `REVERSE` on all four drive motors in `Robot.java`.

**Robot turns or strafes instead of going straight**  
One motor is reversed. Use Drive Test (D-pad) to find which wheel is wrong.

**Intake is backwards**  
Change `intake.setDirection` in `Robot.java`.

**Intake is too fast or too weak**  
Change `INTAKE_SPEED` or `OUTTAKE_SPEED` at the top of `Robot.java`.

**I want a different button**  
Edit `TeleOpMain.java`. That file is only about the controllers.

## How to add a new motor (for example a launcher)

1. Add the motor on the Driver Hub and give it a name, like `launcher`.
2. In `Robot.java`, add:
   - a name constant
   - a `public DcMotor launcher;`
   - `launcher = hardwareMap.get(DcMotor.class, "launcher");`
   - a method like `public void launcherOn() { launcher.setPower(0.8); }`
3. In `TeleOpMain.java`, call that method from a button, for example `if (gamepad1.a) { robot.launcherOn(); }`

## How to change Auto

Open `Auto.java`. It already drives forward for 1 second.

- `robot.drive(0.5, 0, 0);` means forward at half speed. The three numbers are forward, strafe, turn.
- `sleep(1000);` waits 1 second (1000 milliseconds).
- Always call `robot.stop();` when the movement is done.

Test on the field every time you change a number.
