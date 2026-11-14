package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;

/**
 * Match TeleOp: mecanum drive through Pedro's follower, plus intake, transfer,
 * velocity-controlled dual outtake, two-position servo, and field-aimed turret.
 *
 * <p>Gamepad 1 drives; Gamepad 2 runs the scoring mechanisms. Bindings are
 * listed in {@link #init()} telemetry.
 *
 * <p>Hardware: four mecanum motors and Pinpoint from {@link Constants}, plus
 * the intake, outtake, two-position servo, and two turret servos from
 * {@link RobotConstants}.
 *
 * <p>Do not run path-following (home pose) until Pinpoint and Foresight are
 * tuned. Manual drive still works before that if motor directions are right.
 */
@TeleOp(name = "Main TeleOp", group = RobotConstants.OpModeGroups.TELEOP)
public class MainTeleOp extends OpMode {

    /**
     * Set to {@code false} to run TeleOp without the turret. While
     * {@code false} the two turret servos are never touched (no hardware
     * lookup, no power), so TeleOp runs fine without them configured.
     */
    private static final boolean TURRET_ENABLED = true;

    private Follower follower;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private Servo servo;

    private final ToggleButton fieldCentricToggle = new ToggleButton();
    private final ToggleButton homeToggle = new ToggleButton();
    private final ToggleButton speedUpToggle = new ToggleButton();
    private final ToggleButton speedDownToggle = new ToggleButton();
    private final ToggleButton servoToggle = new ToggleButton();
    private final ToggleButton turretHiveToggle = new ToggleButton();
    private final ToggleButton turretCenterToggle = new ToggleButton();
    private final ToggleButton turretHomeToggle = new ToggleButton();

    private boolean fieldCentric;
    private double fieldCentricOffsetRad;
    private boolean drivingToHome;
    private boolean servoAtPositionB;
    private double speedMultiplier = RobotConstants.Drive.DEFAULT_SPEED_MULTIPLIER;

    private long lastLoopNanos;

    /**
     * Builds the follower and intake. Drive commands start in {@link #loop()}
     * because Pedro 3 TeleOp docs use {@code follower.manual()} there, not a
     * separate {@code startTeleopDrive()} call.
     */
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        boolean poseFromAuto = PoseStorage.lastAutoPose != null;
        follower.setPose(poseFromAuto ? PoseStorage.lastAutoPose : RobotConstants.Field.START_POSE);

        fieldCentric = false;
        fieldCentricOffsetRad = 0.0;

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        // Two-position servo, toggled with Gamepad 1 A. Start at POSITION_A.
        servo = hardwareMap.get(Servo.class, RobotConstants.Servo.NAME);
        servoAtPositionB = false;
        servo.setPosition(RobotConstants.Servo.POSITION_A);

        // Turret aims continuously at the hive using the follower's pose, and
        // picks the top or bottom hive side from the robot's Y position. Its
        // angle is only an estimate (no encoder), so the turret must be
        // pointing straight forward right now. Skipped entirely while
        // TURRET_ENABLED is false.
        if (TURRET_ENABLED) {
            turret = new TurretSubsystem(hardwareMap);
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        } else {
            turret = null;
        }

        drivingToHome = false;
        speedMultiplier = RobotConstants.Drive.DEFAULT_SPEED_MULTIPLIER;

        telemetry.addLine("Main TeleOp ready");
        telemetry.addData("Start pose", poseFromAuto ? "FROM AUTO" : "DEFAULT (no auto run)");
        telemetry.addLine("G1: LX/LY drive, RX turn (robot-centric default)");
        telemetry.addLine("G1: Dpad Up/Down = speed +/-  |  X = field-centric  |  LB hold = slow");
        telemetry.addLine("G1: Back = reset field-centric forward  |  Y = drive home  |  A = servo A/B");
        telemetry.addLine("G2: A hold = intake  |  Y hold = reverse intake");
        telemetry.addLine("G2: X hold = transfer  |  B hold = reverse transfer  |  RT hold = outtake  |  LT hold = open gate");
        if (TURRET_ENABLED) {
            telemetry.addLine("G2: Dpad Up = turret HIVE (auto side)  |  Dpad Down = turret CENTER  |  LB = turret HOME");
            telemetry.addLine("Turret must be pointing straight forward now (no encoder)");
        } else {
            telemetry.addLine("Turret: DISABLED (TURRET_ENABLED = false)");
        }
        telemetry.update();
    }

    /**
     * Resets the loop-time and turret clocks when the driver presses Play so
     * the time spent sitting in init does not show up on the first loop.
     */
    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
        if (turret != null) {
            turret.resetAngle();
        }
    }

    /**
     * Reads gamepads, commands drive and intake, then updates hardware.
     */
    @Override
    public void loop() {
        // ---- Read gamepad ----
        // Gamepad stick signs:
        // forward = -left_y (up on the stick is negative in FTC),
        // strafe  = -left_x (left on the stick -> +Y left in Pedro),
        // turn    = -right_x (right on the stick -> clockwise / -heading).
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Adjust speed multiplier with D-pad Up (+10%) and D-pad Down (-10%).
        if (speedUpToggle.update(gamepad1.dpad_up)) {
            speedMultiplier = Math.min(
                    RobotConstants.Drive.MAX_SPEED_MULTIPLIER,
                    speedMultiplier + RobotConstants.Drive.SPEED_STEP
            );
        }
        if (speedDownToggle.update(gamepad1.dpad_down)) {
            speedMultiplier = Math.max(
                    RobotConstants.Drive.MIN_SPEED_MULTIPLIER,
                    speedMultiplier - RobotConstants.Drive.SPEED_STEP
            );
        }

        if (fieldCentricToggle.update(gamepad1.x)) {
            fieldCentric = !fieldCentric;
        }

        // Check for driver cancel BEFORE scaling by activeSpeed so the stick
        // cancel threshold stays the same in slow mode and at low speed settings.
        if (drivingToHome && driverIsMoving(forward, strafe, turn)) {
            drivingToHome = false;
        }

        boolean slowMode = gamepad1.left_bumper;
        double activeSpeed = slowMode ? RobotConstants.Drive.SLOW_MODE_MULTIPLIER : speedMultiplier;
        forward *= activeSpeed;
        strafe *= activeSpeed;
        turn *= activeSpeed;

        // Back (not Start) so we do not fight the Driver Station's Start
        // "emergency" habit. Records the current field heading as field-centric
        // "forward" WITHOUT calling follower.setPose(...) — overwriting the
        // localizer heading would throw off the turret's field aiming.
        if (gamepad1.back) {
            fieldCentricOffsetRad = follower.pose().heading();
        }

        // ---- Optional drive-to-home ----
        if (homeToggle.update(gamepad1.y)) {
            startDriveToHome();
        }

        // ---- Drive ----
        if (drivingToHome) {
            if (!follower.isBusy()) {
                drivingToHome = false;
                follower.manual(0.0, 0.0, 0.0);
            }
        } else {
            commandManualDrive(forward, strafe, turn);
        }

        // ---- Two-position servo (Gamepad 1 A toggles) ----
        if (servoToggle.update(gamepad1.a)) {
            servoAtPositionB = !servoAtPositionB;
            servo.setPosition(servoAtPositionB
                    ? RobotConstants.Servo.POSITION_B
                    : RobotConstants.Servo.POSITION_A);
        }

        // ---- Intake ----
        updateIntakeFromGamepad();

        // ---- Outtake ----
        updateOuttakeFromGamepad();

        // ---- Turret target / override selection ----
        if (turret != null) {
            updateTurretFromGamepad();
        }

        // ---- Hardware write ----
        intake.update();
        outtake.update();
        follower.update();
        // Aim with the freshest localizer pose, every loop, no blocking.
        if (turret != null) {
            turret.update(follower.pose());
        }

        // ---- Telemetry ----
        writeTelemetry(slowMode);
    }

    /**
     * Stops the intake and zeros drive. Safe if init() failed partway.
     */
    @Override
    public void stop() {
        if (intake != null) {
            intake.stop();
            intake.update();
        }
        if (outtake != null) {
            outtake.stop();
            outtake.update();
        }
        if (turret != null) {
            turret.stop();
        }
        if (follower != null) {
            follower.manual(0.0, 0.0, 0.0);
            follower.update();
        }
    }

    /**
     * Starts a straight line from the robot's current pose to
     * {@link RobotConstants.Field#SCORE_POSE}.
     *
     * <p>If the robot is already at {@code SCORE_POSE}, a zero-length line path
     * cannot be constructed, so it simply holds the score pose instead.
     */
    private void startDriveToHome() {
        Pose current = follower.pose();
        Pose home = RobotConstants.Field.SCORE_POSE;

        if (current.distance(home) < 0.5) {
            follower.hold(home);
            drivingToHome = false;
            return;
        }

        Path homePath = line(current, home).linear(current, home);
        follower.follow(homePath);
        drivingToHome = true;
    }

    /**
     * @return {@code true} if any stick is outside the cancel deadzone
     */
    private boolean driverIsMoving(double forward, double strafe, double turn) {
        double deadzone = RobotConstants.Drive.AUTOMATION_CANCEL_DEADZONE;
        return Math.abs(forward) > deadzone
                || Math.abs(strafe) > deadzone
                || Math.abs(turn) > deadzone;
    }

    /**
     * Sends stick values to Pedro using robot-centric or field-centric math.
     *
     * @param forward forward power in [-1, 1], robot-centric before conversion
     * @param strafe  lateral power in [-1, 1], left positive (+Y in Pedro)
     * @param turn    heading power in [-1, 1], counterclockwise positive
     */
    private void commandManualDrive(double forward, double strafe, double turn) {
        if (fieldCentric) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turn,
                    follower.pose().heading(),
                    -fieldCentricOffsetRad
            );
            follower.manual(powers);
        } else {
            follower.manual(forward, strafe, turn);
        }
    }

    /**
     * Gamepad 2 face buttons: A hold = intake, Y hold = reverse intake,
     * X hold = transfer, B hold = reverse transfer.
     */
    private void updateIntakeFromGamepad() {
        if (gamepad2.y) {
            intake.reverse();
        } else if (gamepad2.a) {
            intake.intake();
        } else {
            intake.stopIntake();
        }

        if (gamepad2.x) {
            intake.transfer();
        } else if (gamepad2.b) {
            intake.reverseTransfer();
        } else {
            intake.stopTransfer();
        }
    }

    /**
     * Holding the right trigger spins the outtake up to the fixed
     * {@link RobotConstants.Outtake#TARGET_RPM}. Holding the left trigger
     * opens the outtake gate; releasing it closes the gate.
     */
    private void updateOuttakeFromGamepad() {
        if (gamepad2.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE) {
            outtake.outtake();
        } else {
            outtake.stopMotor();
        }

        outtake.setGateOpen(gamepad2.left_trigger > RobotConstants.Intake.TRIGGER_DEADZONE);
    }

    /**
     * Gamepad 2 turret controls. Dpad Up aims at the hive, with the top or
     * bottom side picked automatically from the robot's Y position. Dpad Down
     * aims at the fixed CENTER target instead. LB toggles a manual HOME
     * override that parks the turret at its starting angle regardless of robot
     * pose. Selecting a target while HOME is active also returns the turret to
     * TRACKING.
     */
    private void updateTurretFromGamepad() {
        if (turretHiveToggle.update(gamepad2.dpad_up)) {
            turret.setAutoSelectSide(true);
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        }
        if (turretCenterToggle.update(gamepad2.dpad_down)) {
            turret.setTargetCoordinate(
                    RobotConstants.Turret.CENTER_TARGET_X_INCHES,
                    RobotConstants.Turret.CENTER_TARGET_Y_INCHES
            );
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        }
        if (turretHomeToggle.update(gamepad2.left_bumper)) {
            boolean goHome = turret.getMode() != TurretSubsystem.TurretMode.HOME;
            turret.setMode(goHome
                    ? TurretSubsystem.TurretMode.HOME
                    : TurretSubsystem.TurretMode.TRACKING);
        }
    }

    /**
     * One telemetry.update() per loop, as required by the Driver Station.
     *
     * @param slowMode whether the slow-mode bumper is held this loop
     */
    private void writeTelemetry(boolean slowMode) {
        long now = System.nanoTime();
        double loopMs = (now - lastLoopNanos) / 1_000_000.0;
        lastLoopNanos = now;

        Pose pose = follower.pose();
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Drive mode", fieldCentric ? "FIELD" : "ROBOT");
        telemetry.addData("Drive speed", "%.0f%%", speedMultiplier * 100.0);
        telemetry.addData("Slow mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Transfer", intake.getTransferState());
        telemetry.addData("Outtake Motor", outtake.getMotorState());
        telemetry.addData("Outtake RPM (L/R)", "%.0f / %.0f (target=%.0f, atSpeed=%b)",
                outtake.getLeftRpm(), outtake.getRightRpm(), outtake.getTargetRpm(), outtake.atSpeed());
        telemetry.addData("Outtake Gate", outtake.isGateOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Servo", servoAtPositionB ? "POSITION_B" : "POSITION_A");
        if (turret != null) {
            telemetry.addData("Turret mode", turret.getMode());
            telemetry.addData("Turret target (in)", "(%.1f, %.1f)", turret.getTargetX(), turret.getTargetY());
            telemetry.addData("Target distance (in)", "%.1f", turret.getDistanceToTarget(pose));
            turret.addTelemetry(telemetry);
        } else {
            telemetry.addData("Turret", "DISABLED");
        }
        telemetry.addData("Drive to home", drivingToHome ? "FOLLOWING" : "OFF");
        telemetry.addData("Loop (ms)", loopMs);
        telemetry.update();
    }
}
