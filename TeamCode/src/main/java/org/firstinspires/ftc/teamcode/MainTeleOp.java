package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;

/**
 * Match TeleOp: mecanum drive through Pedro's follower, plus the intake.
 *
 * <p>Gamepad 1 drives and runs the intake so a single driver can test the
 * robot. Bindings are listed in {@link #init()} telemetry.
 *
 * <p>Hardware: four mecanum motors and Pinpoint from {@link Constants}, plus
 * the intake motor from {@link RobotConstants.Intake}.
 *
 * <p>Do not run path-following (home pose) until Pinpoint and Foresight are
 * tuned. Manual drive still works before that if motor directions are right.
 */
@TeleOp(name = "Main TeleOp", group = RobotConstants.OpModeGroups.TELEOP)
public class MainTeleOp extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;

    private final ToggleButton fieldCentricToggle = new ToggleButton();
    private final ToggleButton intakeLatchToggle = new ToggleButton();
    private final ToggleButton homeToggle = new ToggleButton();

    private boolean fieldCentric;
    private boolean intakeLatched;
    private boolean drivingToHome;

    private long lastLoopNanos;

    /**
     * Builds the follower and intake. Drive commands start in {@link #loop()}
     * because Pedro 3 TeleOp docs use {@code follower.manual()} there, not a
     * separate {@code startTeleopDrive()} call.
     */
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        fieldCentric = false;
        intakeLatched = false;
        drivingToHome = false;

        telemetry.addLine("Main TeleOp ready");
        telemetry.addLine("LX/LY drive, RX turn (robot-centric default)");
        telemetry.addLine("LB hold = slow  |  X = field-centric  |  Back = reset heading");
        telemetry.addLine("RT intake  |  LT reverse  |  A latch intake  |  Y drive home");
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
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        boolean slowMode = gamepad1.left_bumper;
        if (slowMode) {
            forward *= RobotConstants.Drive.SLOW_MODE_MULTIPLIER;
            strafe *= RobotConstants.Drive.SLOW_MODE_MULTIPLIER;
            turn *= RobotConstants.Drive.SLOW_MODE_MULTIPLIER;
        }

        if (fieldCentricToggle.update(gamepad1.x)) {
            fieldCentric = !fieldCentric;
        }

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
            }
        } else {
            commandManualDrive(forward, strafe, turn);
        }

        // ---- Intake ----
        updateIntakeFromGamepad();

        // ---- Hardware write ----
        intake.update();
        follower.update();

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
        if (follower != null) {
            follower.manual(0.0, 0.0, 0.0);
            follower.update();
        }
    }

    /**
     * Starts a straight line to the placeholder home pose.
     *
     * <p>Pedro coordinates: x/y inches, heading radians. The pose in
     * {@link RobotConstants.Field} is a TODO until you measure a real spot.
     */
    private void startDriveToHome() {
        Pose home = new Pose(
                RobotConstants.Field.HOME_X_INCHES,
                RobotConstants.Field.HOME_Y_INCHES,
                RobotConstants.Field.HOME_HEADING_RADIANS
        );
        // TODO(team): replace HOME_* with a real field pose after Pinpoint is tuned
        Path homePath = line(follower.pose(), home).linear(follower.pose(), home);
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
     * Triggers are momentary. A latches continuous intake. Reverse wins if
     * both triggers are held so a stuck latch can still spit a jam.
     */
    private void updateIntakeFromGamepad() {
        if (intakeLatchToggle.update(gamepad1.a)) {
            intakeLatched = !intakeLatched;
        }

        boolean reverseHeld = gamepad1.left_trigger > RobotConstants.Intake.TRIGGER_DEADZONE;
        boolean intakeHeld = gamepad1.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE;

        if (reverseHeld) {
            intake.reverse();
        } else if (intakeHeld || intakeLatched) {
            intake.intake();
        } else {
            intake.stop();
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
        telemetry.addData("Slow mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Drive to home", drivingToHome ? "FOLLOWING" : "OFF");
        telemetry.addData("Loop (ms)", loopMs);
        telemetry.update();
    }
}
