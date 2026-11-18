package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

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
@TeleOp(name = RobotConstants.OpModeNames.MAIN_TELEOP, group = RobotConstants.OpModeGroups.TELEOP)
public class MainTeleOp extends OpMode {

    /**
     * Set to {@code false} to run TeleOp without the turret. While
     * {@code false} the two turret servos and the turret encoder are never
     * touched (no hardware lookup, no power), so TeleOp runs fine without
     * them configured.
     */
    private static final boolean TURRET_ENABLED = true;

    /**
     * Set to {@code true} to bring back the two-position {@code flower} servo
     * on Gamepad 1 A. While {@code false} the servo is never touched (no
     * hardware lookup, no position command), so TeleOp runs fine without it
     * configured.
     */
    private static final boolean FLOWER_SERVO_ENABLED = false;

    private Follower follower;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private Servo servo;

    private final ToggleButton fieldCentricToggle = new ToggleButton();
    private final ToggleButton homeToggle = new ToggleButton();
    private final ToggleButton opposeToggle = new ToggleButton();
    private final ToggleButton speedUpToggle = new ToggleButton();
    private final ToggleButton speedDownToggle = new ToggleButton();
    private final ToggleButton servoToggle = new ToggleButton();
    private final ToggleButton turretHiveToggle = new ToggleButton();
    private final ToggleButton turretForwardToggle = new ToggleButton();
    private final ToggleButton turretHomeToggle = new ToggleButton();
    /** True while Gamepad 2 Dpad Left/Right was jogging the turret on the last loop. */
    private boolean turretJogging;
    private final ToggleButton turretZeroToggle = new ToggleButton();
    private final ToggleButton poseDefaultToggle = new ToggleButton();
    private final ToggleButton poseAutoToggle = new ToggleButton();
    private final ToggleButton poseResumeToggle = new ToggleButton();

    private boolean fieldCentric;
    private double fieldCentricOffsetRad;
    private boolean drivingToHome;
    private boolean servoAtPositionB;
    private boolean turretAtSideAngle;
    private boolean wasShootTrigger;
    private Command activeShoot;
    private double speedMultiplier = RobotConstants.Drive.DEFAULT_SPEED_MULTIPLIER;

    private long lastLoopNanos;

    /** Pose the follower was told it starts at, and where that pose came from. */
    private Pose startPose;
    private String startPoseSource = "";

    /**
     * Where the turret zero came from if it was NOT taken on this init screen
     * ("the last autonomous" or "the last TeleOp"), or {@code null} if the turret
     * was zeroed where it points now.
     */
    private String turretZeroKeptFrom;

    /**
     * Builds the follower and intake. Drive commands start in {@link #loop()}
     * because Pedro 3 TeleOp docs use {@code follower.manual()} there, not a
     * separate {@code startTeleopDrive()} call.
     */
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);

        // Start from where autonomous ended if one just ran, otherwise from the
        // default start pose. The driver can change this on the init screen
        // (see init_loop) because the turret aims from this pose all match.
        Pose autoPose = RobotBlackboard.getPose(blackboard, RobotBlackboard.LAST_AUTO_POSE);
        if (autoPose != null) {
            applyStartPose(autoPose, "FROM AUTO");
        } else {
            // No unused autonomous pose: none has run, or a TeleOp already used it.
            applyStartPose(RobotConstants.Field.START_POSE, "DEFAULT START (no saved auto pose)");
        }

        fieldCentric = false;
        fieldCentricOffsetRad = 0.0;

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        // Two-position servo, toggled with Gamepad 1 A. Start at POSITION_A.
        // Skipped entirely while FLOWER_SERVO_ENABLED is false.
        servoAtPositionB = false;
        if (FLOWER_SERVO_ENABLED) {
            servo = hardwareMap.get(Servo.class, RobotConstants.Servo.NAME);
            servo.setPosition(RobotConstants.Servo.POSITION_A);
        } else {
            servo = null;
        }

        // Turret aims continuously at the hive using the follower's pose, and
        // picks the top or bottom hive side from the robot's Y position. The
        // constructor zeroes the through-bore encoder here. That zero is only
        // kept when no autonomous has just run (see below); then the turret
        // must be pointing straight forward right now (or press Gamepad 2 Y on
        // the init screen once it is). Must be built AFTER the intake, because
        // the encoder is read through the intake motor's port. Skipped
        // entirely while TURRET_ENABLED is false.
        turretZeroKeptFrom = null;
        if (TURRET_ENABLED) {
            turret = new TurretSubsystem(hardwareMap);
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        } else {
            turret = null;
        }
        // Coming straight from autonomous: keep the turret zero that autonomous
        // recorded instead of the one just taken, because the turret may have
        // moved during autonomous and nobody may touch the robot in between.
        if (autoPose != null) {
            restoreTurretZero(
                    RobotBlackboard.getInteger(blackboard, RobotBlackboard.LAST_AUTO_TURRET_ZERO_TICKS),
                    "the last autonomous"
            );
        }

        drivingToHome = false;
        speedMultiplier = RobotConstants.Drive.DEFAULT_SPEED_MULTIPLIER;

        writeInitTelemetry();
    }

    /**
     * Runs repeatedly between Init and Play. Nothing moves here: the turret
     * servos are still unpowered, so the turret can be turned by hand.
     *
     * <ul>
     *   <li>Shows the start pose the turret will aim from, and lets Gamepad 1
     *       pick a different one. Each choice brings its turret zero with it:
     *       Dpad Left = default start pose, turret zeroed where it points now;
     *       Dpad Up = pose and turret zero from the last autonomous; Dpad Right
     *       = pose and turret zero from the last TeleOp (for a restart in the
     *       middle of a match).</li>
     *   <li>Shows the LIVE turret angle and encoder ticks, so the encoder
     *       direction and the zero can be checked before the servos get
     *       power. Gamepad 2 Y re-zeroes the turret where it points now.</li>
     * </ul>
     */
    @Override
    public void init_loop() {
        // Each start-pose choice brings its matching turret zero with it.
        if (poseDefaultToggle.update(gamepad1.dpad_left)) {
            // Robot was placed by hand on the start tile: zero the turret where it
            // points now (it must be straight forward).
            applyStartPose(RobotConstants.Field.START_POSE, "DEFAULT START (driver chose)");
            zeroTurretHere();
        }
        Pose autoPose = RobotBlackboard.getPose(blackboard, RobotBlackboard.LAST_AUTO_POSE);
        if (poseAutoToggle.update(gamepad1.dpad_up) && autoPose != null) {
            applyStartPose(autoPose, "FROM AUTO");
            restoreTurretZero(
                    RobotBlackboard.getInteger(blackboard, RobotBlackboard.LAST_AUTO_TURRET_ZERO_TICKS),
                    "the last autonomous"
            );
        }
        Pose teleOpPose = RobotBlackboard.getPose(blackboard, RobotBlackboard.LAST_TELEOP_POSE);
        if (poseResumeToggle.update(gamepad1.dpad_right) && teleOpPose != null) {
            applyStartPose(teleOpPose, "LAST TELEOP (driver chose)");
            // In a mid-match restart nobody can straighten the turret by hand, so
            // it is probably NOT pointing forward now. The hub kept counting while
            // TeleOp was stopped, so bring back the zero that TeleOp was using.
            // Gamepad 2 Y still overrides it.
            restoreTurretZero(
                    RobotBlackboard.getInteger(blackboard, RobotBlackboard.LAST_TELEOP_TURRET_ZERO_TICKS),
                    "the last TeleOp"
            );
        }

        if (turret != null) {
            if (turretZeroToggle.update(gamepad2.y)) {
                zeroTurretHere();
            }
            // Read-only: updates the angle and ticks without powering the servos.
            turret.refresh();
        }

        writeInitTelemetry();
    }

    /** Declares the turret's current direction to be straight forward. */
    private void zeroTurretHere() {
        if (turret != null) {
            turret.resetAngle();
            turretZeroKeptFrom = null;
        }
    }

    /**
     * Brings back a turret zero saved by an earlier OpMode, so the turret does not
     * have to be pointing forward now. Does nothing if there is no saved zero.
     *
     * @param zeroTicks saved encoder count that means straight forward, or null
     * @param from      short label shown on the Driver Station
     */
    private void restoreTurretZero(Integer zeroTicks, String from) {
        if (turret != null && zeroTicks != null) {
            turret.restoreZero(zeroTicks);
            turretZeroKeptFrom = from;
        }
    }

    /**
     * Tells the follower (and so the turret) where the robot is starting.
     *
     * @param pose   field pose to start from
     * @param source short label shown on the Driver Station
     */
    private void applyStartPose(Pose pose, String source) {
        follower.setPose(pose);
        startPose = pose;
        startPoseSource = source;
    }

    /**
     * Init screen: start pose, live turret readout, then the button list.
     * Called every init_loop because telemetry is cleared on each update.
     */
    private void writeInitTelemetry() {
        telemetry.addLine("Main TeleOp ready");
        telemetry.addData("Start pose", startPoseSource);
        telemetry.addData("Start X / Y / Heading", "%.1f in / %.1f in / %.1f deg",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addLine("  Is the robot REALLY there? If not: G1 Dpad Left = default start"
                + (turret != null ? " + zero turret HERE" : "")
                + (RobotBlackboard.getPose(blackboard, RobotBlackboard.LAST_AUTO_POSE) != null
                        ? "  |  Dpad Up = auto pose + turret zero" : "")
                + (RobotBlackboard.getPose(blackboard, RobotBlackboard.LAST_TELEOP_POSE) != null
                        ? "  |  Dpad Right = last TeleOp pose + turret zero" : ""));
        if (turret != null) {
            telemetry.addData("Turret measuredAngleDeg (live)", "%.1f", turret.getCurrentAngleDegrees());
            telemetry.addData("Turret encoderTicks (live)", turret.getEncoderTicks());
            telemetry.addLine(turretZeroKeptFrom != null
                    ? "  Turret zero KEPT from " + turretZeroKeptFrom
                            + ": a non-zero angle is correct. G2 Y = zero it here instead."
                    : "  Turret must point straight forward and read 0.0. G2 Y = zero it here.");
            telemetry.addLine("  Check 1: turn the TURRET LEFT by hand (counterclockwise from above), angle must go UP."
                    + " If it goes DOWN, flip Turret.ENCODER_REVERSED.");
            telemetry.addLine("  Check 2: turn the ROBOT LEFT by hand, Heading (deg) after Play must go UP.");
            telemetry.addLine("  The turret starts moving by itself when you press Play. Hands clear.");
        } else {
            telemetry.addLine("Turret: DISABLED (TURRET_ENABLED = false)");
        }
        telemetry.addLine("G1: LX/LY drive, RX turn (robot-centric)");
        telemetry.addLine("G1: Dpad Up/Down = speed +/-  |  LB hold = slow");
        telemetry.addLine("G1: X = drive to start pose  |  B = drive to opposite pose  |  any stick cancels"
                + (FLOWER_SERVO_ENABLED ? "  |  A = servo A/B" : ""));
        telemetry.addLine("G2: A hold = intake  |  Y hold = reverse intake");
        telemetry.addLine("G2: X hold = transfer  |  B hold = reverse transfer");
        telemetry.addLine("G2: LT hold = open gate  |  RT press = shoot balls (waits for the turret to aim)");
        telemetry.addLine("Outtake flywheels run the whole time after Play");
        if (turret != null) {
            telemetry.addLine("Turret aims at the HIVE by itself (top/bottom side picked automatically)");
            telemetry.addLine("G2: Dpad Up = turret faces straight ahead; press again = back to the hive");
            telemetry.addLine("G2: Dpad Down = back to the hive");
            telemetry.addLine("G2: Dpad Left/Right HOLD = jog turret left/right, holds where released"
                    + " (Dpad LEFT must turn it LEFT and make the angle go UP)");
            telemetry.addLine("G2: LB = turret 90 deg left / straight forward (Dpad Down to aim again)");
        }
        telemetry.addData("Gamepad 2", gamepad2.getGamepadId() == Gamepad.ID_UNASSOCIATED
                ? "NOT CONNECTED (press Start+B on it)" : "connected");
        telemetry.update();
    }

    /**
     * Resets the loop timer when the driver presses Play so the time spent
     * sitting in init does not show up on the first loop.
     *
     * <p>The turret is NOT re-zeroed here. Its zero came from the autonomous
     * that just ran, or was taken at Init or with Gamepad 2 Y on the init
     * screen, or was restored with Gamepad 1 Dpad Right. The encoder has been
     * counting ever since, so a turret that moved in between is still measured
     * correctly.
     */
    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
        // The autonomous pose has now been used. Clear it so a later practice
        // TeleOp does not silently start from an old autonomous end pose.
        blackboard.remove(RobotBlackboard.LAST_AUTO_POSE);
        blackboard.remove(RobotBlackboard.LAST_AUTO_TURRET_ZERO_TICKS);
        // Remember the turret zero this TeleOp runs with, next to the TeleOp pose,
        // so a restart in the middle of a match can restore both.
        if (turret != null) {
            blackboard.put(
                    RobotBlackboard.LAST_TELEOP_TURRET_ZERO_TICKS,
                    turret.getZeroOffsetTicks()
            );
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

        // ---- Optional drive-to-pose (each button needs its OWN edge detector) ----
        // Gamepad 1 X: drive to the start pose.
        if (homeToggle.update(gamepad1.x)) {
            startDriveToStart();
        }

        // Gamepad 1 B: drive to the opposite start pose.
        if (opposeToggle.update(gamepad1.b)) {
            startDriveToOppose();
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
        if (servo != null && servoToggle.update(gamepad1.a)) {
            servoAtPositionB = !servoAtPositionB;
            servo.setPosition(servoAtPositionB
                    ? RobotConstants.Servo.POSITION_B
                    : RobotConstants.Servo.POSITION_A);
        }

        // ---- Shoot balls (Gamepad 2 RT) ----
        // Ivy sequence built in shootBalls() below (it also waits for the turret
        // to aim; SecondAuto has its own version). Rising edge starts it; it runs
        // to completion. Face buttons and LT are ignored while it is active.
        boolean shootPressed = gamepad2.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE;
        boolean shooting = isShootActive();
        if (shootPressed && !wasShootTrigger && !shooting) {
            activeShoot = shootBalls();
            schedule(activeShoot);
            shooting = true;
        }
        wasShootTrigger = shootPressed;

        // ---- Intake / outtake ----
        // Flywheels stay on for the whole TeleOp, including during shoot.
        outtake.outtake();
        if (!shooting) {
            updateIntakeFromGamepad();
            outtake.setGateOpen(gamepad2.left_trigger > RobotConstants.Intake.TRIGGER_DEADZONE);
        }

        // ---- Turret target / override selection ----
        if (turret != null) {
            updateTurretFromGamepad();
        }

        // ---- Hardware write ----
        Scheduler.execute();
        intake.update();
        outtake.update();
        follower.update();
        // Remember where we are, so a TeleOp restart in the middle of a match
        // can resume from here (Gamepad 1 Dpad Right on the init screen).
        blackboard.put(RobotBlackboard.LAST_TELEOP_POSE, follower.pose());
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
        Scheduler.reset();
        activeShoot = null;
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
     * {@link RobotConstants.Field#START_POSE}.
     *
     * <p>If the robot is already there, a zero-length line path cannot be
     * constructed, so it simply holds that pose instead.
     */
    private void startDriveToStart() {
        Pose current = follower.pose();
        Pose home = RobotConstants.Field.START_POSE;

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
     * Starts a straight line from the robot's current pose to
     * {@link RobotConstants.Field#OPPOSE_POSE} (the start pose at the other
     * end of the field).
     */
    private void startDriveToOppose() {
        Pose current = follower.pose();
        Pose home = RobotConstants.Field.OPPOSE_POSE;

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
     * TeleOp shoot sequence: wait for the already-running flywheels to be at
     * speed, wait for the turret to finish aiming, open the gate, feed with
     * intake and transfer, then stop the feed and close the gate. The
     * flywheels are left running.
     *
     * <p>Both waits have a timeout, so a slow flywheel or a turret that never
     * settles delays the shot but cannot block it forever.
     */
    private Command shootBalls() {
        return sequential(
                race(
                        waitUntil(() -> outtake.atSpeed()),
                        waitMs(RobotConstants.Auto.SPIN_UP_TIMEOUT_MS)
                ),
                race(
                        waitUntil(() -> turretReadyToShoot()),
                        waitMs(RobotConstants.Turret.AIM_WAIT_TIMEOUT_MS)
                ),
                instant(() -> outtake.setGateOpen(true)),
                waitMs(RobotConstants.Auto.GATE_OPEN_WAIT_MS),
                instant(() -> {
                    intake.intake();
                    intake.transfer();
                }),
                waitMs(RobotConstants.Auto.FEED_TIME_MS),
                instant(() -> {
                    intake.stop();
                    outtake.setGateOpen(false);
                })
        );
    }

    /**
     * @return {@code true} if the shot does not need to wait for the turret:
     *         there is no turret, the turret is aimed at its target, or the
     *         operator has deliberately parked it (HOME / DISABLED) and is
     *         aiming with the whole robot instead
     */
    private boolean turretReadyToShoot() {
        return turret == null
                || turret.getMode() != TurretSubsystem.TurretMode.TRACKING
                || turret.isAimedAtTarget();
    }

    /**
     * @return {@code true} if the shoot-balls Ivy command is still scheduled
     *         or running
     */
    private boolean isShootActive() {
        return activeShoot != null
                && (Scheduler.isScheduled(activeShoot) || Scheduler.isRunning(activeShoot));
    }

    /**
     * Gamepad 2 turret controls. The hive is the only field target.
     *
     * <p>The turret aims at the hive (TRACKING) from Play on, with the top or bottom
     * side picked from the robot's Y position. Every button below that leaves the hive
     * is undone by pressing it again or by Dpad Down.
     *
     * <ul>
     *   <li>Dpad Up: face straight ahead in the robot's heading (HOME at 0 degrees).
     *       Press again to go back to the hive.</li>
     *   <li>Dpad Down: back to the hive. Also clears an encoder fault and retries.</li>
     *   <li>Dpad Left / Right (hold): jog the turret left / right at
     *       {@link RobotConstants.Turret#JOG_POWER}; on release it holds the angle it
     *       stopped at. Dpad LEFT must turn the turret LEFT (counterclockwise from
     *       above) and make measuredAngleDeg go UP: that is the direction check.</li>
     *   <li>LB: stop tracking and turn 90 degrees
     *       ({@link RobotConstants.Turret#SIDE_ANGLE_RADIANS}) from straight forward;
     *       press again to turn back to straight forward.</li>
     * </ul>
     */
    private void updateTurretFromGamepad() {
        if (turretForwardToggle.update(gamepad2.dpad_up)) {
            boolean facingForward = turret.getMode() == TurretSubsystem.TurretMode.HOME
                    && !turretAtSideAngle
                    && turret.getHomeAngleRadians() == RobotConstants.Turret.HOME_ANGLE_RADIANS;
            if (facingForward) {
                aimTurretAtHive();
            } else {
                turret.setHomeAngle(RobotConstants.Turret.HOME_ANGLE_RADIANS);
                turret.setMode(TurretSubsystem.TurretMode.HOME);
                turretAtSideAngle = false;
            }
        }
        if (turretHiveToggle.update(gamepad2.dpad_down)) {
            turret.clearFault();
            aimTurretAtHive();
        }

        // Jog: held buttons, not edges. Left = +power (counterclockwise).
        boolean jogLeft = gamepad2.dpad_left;
        boolean jogRight = gamepad2.dpad_right;
        if (jogLeft != jogRight) {
            turret.jog(jogLeft ? RobotConstants.Turret.JOG_POWER : -RobotConstants.Turret.JOG_POWER);
            turretJogging = true;
            turretAtSideAngle = false;
        } else if (turretJogging) {
            // Both released (or both held): stop and hold the angle reached.
            turret.holdCurrentAngle();
            turretJogging = false;
        }

        if (turretHomeToggle.update(gamepad2.left_bumper)) {
            // First press (or first press after tracking): turn to the side
            // angle. Next press: turn back to straight forward. Repeats.
            boolean holdingFixedAngle = turret.getMode() == TurretSubsystem.TurretMode.HOME;
            turretAtSideAngle = !(holdingFixedAngle && turretAtSideAngle);
            turret.setHomeAngle(turretAtSideAngle
                    ? RobotConstants.Turret.SIDE_ANGLE_RADIANS
                    : RobotConstants.Turret.HOME_ANGLE_RADIANS);
            turret.setMode(TurretSubsystem.TurretMode.HOME);
        }
    }

    /** Puts the turret back on the hive, with automatic top/bottom side selection. */
    private void aimTurretAtHive() {
        turret.setAutoSelectSide(true);
        turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        turretAtSideAngle = false;
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
        telemetry.addData("Start pose was", startPoseSource);
        telemetry.addData("Drive mode", fieldCentric ? "FIELD" : "ROBOT");
        telemetry.addData("Drive speed", "%.0f%%", speedMultiplier * 100.0);
        telemetry.addData("Slow mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Transfer", intake.getTransferState());
        telemetry.addData("Outtake Motor", outtake.getMotorState());
        telemetry.addData("Outtake RPM (L/R)", "%.0f / %.0f (target=%.0f, atSpeed=%b)",
                outtake.getLeftRpm(), outtake.getRightRpm(), outtake.getTargetRpm(), outtake.atSpeed());
        telemetry.addData("Outtake Gate", outtake.isGateOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Shoot balls", isShootActive() ? "RUNNING" : "OFF");
        telemetry.addData("Servo", servo == null
                ? "DISABLED"
                : (servoAtPositionB ? "POSITION_B" : "POSITION_A"));
        telemetry.addData("Gamepad 2", gamepad2.getGamepadId() == Gamepad.ID_UNASSOCIATED
                ? "NOT CONNECTED (press Start+B on it)" : "connected");
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
