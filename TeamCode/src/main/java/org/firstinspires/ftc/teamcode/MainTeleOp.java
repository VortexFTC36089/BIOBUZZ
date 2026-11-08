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
 * Match TeleOp: mecanum drive through Pedro's follower, plus the intake.
 *
 * <p>Gamepad 1 drives and runs the intake so a single driver can test the
 * robot. Bindings are listed in {@link #init()} telemetry.
 *
 * <p>Hardware: four mecanum motors and Pinpoint from {@link Constants}, plus
 * the intake motor and two-position servo from {@link RobotConstants}.
 *
 * <p>Do not run path-following (home pose) until Pinpoint and Foresight are
 * tuned. Manual drive still works before that if motor directions are right.
 */
@TeleOp(name = "Main TeleOp", group = RobotConstants.OpModeGroups.TELEOP)
public class MainTeleOp extends OpMode {

    /**
     * Set to {@code true} to build and run the turret in TeleOp. While
     * {@code false} the turret motor is never touched (no hardware lookup,
     * no power), so TeleOp runs fine without {@code turretMotor} configured.
     */
    private static final boolean TURRET_ENABLED = false;

    private Follower follower;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private Servo servo;

    private final ToggleButton fieldCentricToggle = new ToggleButton();
    private final ToggleButton intakeLatchToggle = new ToggleButton();
    private final ToggleButton homeToggle = new ToggleButton();
    private final ToggleButton speedUpToggle = new ToggleButton();
    private final ToggleButton speedDownToggle = new ToggleButton();
    private final ToggleButton servoToggle = new ToggleButton();
    private final ToggleButton turretHighGoalToggle = new ToggleButton();
    private final ToggleButton turretCenterToggle = new ToggleButton();
    private final ToggleButton turretHomeToggle = new ToggleButton();

    private boolean fieldCentric;
    private boolean intakeLatched;
    private boolean drivingToHome;
    private boolean servoAtPositionB;
    private double speedMultiplier = RobotConstants.Drive.DEFAULT_SPEED_MULTIPLIER;

    private double lastLoopNanos;

    /**
     * Builds the follower and intake. Drive commands start in {@link #loop()}
     * because Pedro 3 TeleOp docs use {@code follower.manual()} there, not a
     * separate {@code startTeleopDrive()} call.
     */
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        // Pick up the pose where autonomous ended so TeleOp knows where the
        // robot is on the field. If no auto has run yet, fall back to START_POSE.
        boolean poseFromAuto = PoseStorage.lastAutoPose != null;
        follower.setPose(poseFromAuto ? PoseStorage.lastAutoPose : RobotConstants.Field.START_POSE);

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        // Turret aims continuously using the follower's pose. Start on the
        // default (High Goal) target in TRACKING mode. Skipped entirely while
        // TURRET_ENABLED is false.
        if (TURRET_ENABLED) {
            turret = new TurretSubsystem(hardwareMap);
            turret.setTargetCoordinate(
                    RobotConstants.Turret.DEFAULT_TARGET_X_INCHES,
                    RobotConstants.Turret.DEFAULT_TARGET_Y_INCHES
            );
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        } else {
            turret = null;
        }

        drivingToHome = false;
        speedMultiplier = RobotConstants.Drive.DEFAULT_SPEED_MULTIPLIER;

        telemetry.addLine("Main TeleOp ready");
        telemetry.addData("Start pose", poseFromAuto ? "FROM AUTO" : "DEFAULT (no auto run)");
        telemetry.addLine("G1: LX/LY drive, RX turn (robot-centric default)");
        telemetry.addLine("G1: Dpad Up/Down = speed +/-  |  Back = reset heading  |  Y drive home");
        telemetry.addLine("G2: A hold = intake  |  Y hold = reverse intake");
        telemetry.addLine("G2: X hold = transfer  |  B hold = reverse transfer  |  RT hold = outtake");
        if (TURRET_ENABLED) {
            telemetry.addLine("G2: Dpad Up = turret HIGH GOAL  |  Dpad Down = turret CENTER  |  LB = turret HOME");
        } else {
            telemetry.addLine("Turret: DISABLED (TURRET_ENABLED = false)");
        }
        telemetry.update();
    }

    /**
     * Resets the loop-time clock when the driver presses Play.
     */
    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
    }

    /**
     * Reads gamepads, commands drive and intake, then updates hardware.
     */
    @Override
    public void loop() {
        // ---- Read gamepad ----
        // Official Pedro 3 TeleOp signs (pedropathing.com teleop-usage):
        // forward = -left_y (up on the stick is negative in FTC),
        // strafe  = +left_x (right is positive; Pedro's +Y is left, and
        //            ManualDrive / mecanum handle that internally),
        // turn    = +right_x (right stick right = clockwise / negative
        //            heading in FTC... Pedro documents this un-negated).
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

        boolean slowMode = false;
        double activeSpeed = slowMode ? RobotConstants.Drive.SLOW_MODE_MULTIPLIER : speedMultiplier;
        forward *= activeSpeed;
        strafe *= activeSpeed;
        turn *= activeSpeed;

        // Back (not Start) so we do not fight the Driver Station's Start
        // "emergency" habit. Sets heading to 0 while keeping x/y, so
        // field-centric "forward" becomes the way the robot is facing now.
        if (gamepad1.back) {
            Pose pose = follower.pose();
            follower.setPose(new Pose(pose.x(), pose.y(), 0.0));
        }

        // ---- Optional drive-to-home ----
        if (homeToggle.update(gamepad1.y)) {
            startDriveToHome();
        }
        if (drivingToHome && driverIsMoving(forward, strafe, turn)) {
            drivingToHome = false;
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
     * @param strafe  lateral power in [-1, 1], right positive
     * @param turn    heading power in [-1, 1], right positive
     */
    private void commandManualDrive(double forward, double strafe, double turn) {
        if (fieldCentric) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turn,
                    follower.pose().heading()
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
     * Holding the right trigger spins the outtake up to target RPM.
     */
    private void updateOuttakeFromGamepad() {
        if (gamepad2.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE) {
            outtake.outtake();
        } else {
            outtake.stopMotor();
        }
    }

    /**
     * Gamepad 2 turret controls. Dpad Up / Down pick the field target the turret
     * continuously aims at; LB toggles a manual HOME override that parks the
     * turret at its starting angle regardless of robot pose. Selecting a
     * target while HOME is active also returns the turret to TRACKING.
     */
    private void updateTurretFromGamepad() {
        if (turretHighGoalToggle.update(gamepad2.dpad_up)) {
            turret.setTargetCoordinate(
                    RobotConstants.Turret.DEFAULT_TARGET_X_INCHES,
                    RobotConstants.Turret.DEFAULT_TARGET_Y_INCHES
            );
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
        telemetry.addData("Servo", servoAtPositionB ? "POSITION_B" : "POSITION_A");
        if (turret != null) {
            telemetry.addData("Turret mode", turret.getMode());
            telemetry.addData("Turret target (in)", "(%.1f, %.1f)", turret.getTargetX(), turret.getTargetY());
            telemetry.addData("Turret angle (deg)", "%.1f -> %.1f",
                    turret.getCurrentAngleDegrees(), turret.getTargetAngleDegrees());
            telemetry.addData("Turret error (deg)", "%.2f (onTarget=%b)",
                    turret.getAngleErrorDegrees(), turret.isOnTarget());
            telemetry.addData("Turret power", "%.2f", turret.getLastCommandedPower());
        } else {
            telemetry.addData("Turret", "DISABLED");
        }
        telemetry.addData("Drive to home", drivingToHome ? "FOLLOWING" : "OFF");
        telemetry.addData("Loop (ms)", loopMs);
        telemetry.update();
    }
}
