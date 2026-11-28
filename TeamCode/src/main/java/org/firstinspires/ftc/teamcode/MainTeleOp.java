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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;

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
 * listed in {@link #init()} telemetry. Buttons use their PS5 DualSense names.
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
    private static final boolean TURRET_ENABLED = false;

    /**
     * Set to {@code true} to bring back the {@code flower} servo on Gamepad 1
     * (Cross = positions A/B, hold R2 = twitch). While {@code false} the servo is never touched (no
     * hardware lookup, no position command), so TeleOp runs fine without it
     * configured.
     */
    private static final boolean FLOWER_SERVO_ENABLED = true;

    /**
     * A drive-to-pose spot with its own flywheel RPM and Kp, set in
     * {@link RobotConstants.ShotSpots}. The values are read every loop so Panels edits
     * apply right away.
     */
    private enum ShotSpot {
        BOTTOM_ANGLE_SHOOT(RobotConstants.Field.BOTTOM_ANGLE_SHOOT,
                () -> RobotConstants.ShotSpots.BOTTOM_ANGLE_SHOOT_RPM,
                () -> RobotConstants.ShotSpots.BOTTOM_ANGLE_SHOOT_KP),
        TOP_ANGLE_SHOOT(RobotConstants.Field.TOP_ANGLE_SHOOT,
                () -> RobotConstants.ShotSpots.TOP_ANGLE_SHOOT_RPM,
                () -> RobotConstants.ShotSpots.TOP_ANGLE_SHOOT_KP),
        BOTTOM_SHOOT(RobotConstants.Field.BOTTOM_SHOOT,
                () -> RobotConstants.ShotSpots.BOTTOM_SHOOT_RPM,
                () -> RobotConstants.ShotSpots.BOTTOM_SHOOT_KP),
        OPPOSE_POSE(RobotConstants.Field.OPPOSE_POSE,
                () -> RobotConstants.ShotSpots.OPPOSE_POSE_RPM,
                () -> RobotConstants.ShotSpots.OPPOSE_POSE_KP);

        final Pose pose;
        final DoubleSupplier rpm;
        final DoubleSupplier kp;

        ShotSpot(Pose pose, DoubleSupplier rpm, DoubleSupplier kp) {
            this.pose = pose;
            this.rpm = rpm;
            this.kp = kp;
        }
    }

    private Follower follower;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private Servo servo;

    private final ToggleButton fieldCentricToggle = new ToggleButton();
    private final ToggleButton homeToggle = new ToggleButton();
    private final ToggleButton opposeToggle = new ToggleButton();
    private final ToggleButton topAngleShootToggle = new ToggleButton();
    private final ToggleButton bottomAngleShootToggle = new ToggleButton();
    private final ToggleButton speedUpToggle = new ToggleButton();
    private final ToggleButton speedDownToggle = new ToggleButton();
    private final ToggleButton backToggle = new ToggleButton();
    private final ToggleButton servoToggle = new ToggleButton();
    private final ToggleButton turretHiveToggle = new ToggleButton();
    private final ToggleButton turretForwardToggle = new ToggleButton();
    private final ToggleButton alignToggle = new ToggleButton();
    /** Turns the chassis toward the hive while {@link #aligning} (Gamepad 1 Triangle). */
    private final ChassisAligner chassisAligner = new ChassisAligner();
    private boolean aligning;
    /** True while chassis align is holding the turret straight and must give it back. */
    private boolean turretHeldByAlign;
    private TurretSubsystem.TurretMode turretModeBeforeAlign;
    /** True while Gamepad 2 Dpad Left/Right was jogging the turret on the last loop. */
    private boolean turretJogging;
    /**
     * Every motor port and its encoder count at Init. The init screen uses it to show
     * which port's count changes when the turret is turned by hand, which is the port
     * {@link RobotConstants.Turret#ENCODER_PORT} must name.
     */
    private final Map<String, DcMotorEx> encoderPorts = new LinkedHashMap<>();
    private final Map<String, Integer> encoderPortStartTicks = new HashMap<>();

    private boolean fieldCentric;
    private double fieldCentricOffsetRad;
    private boolean drivingToPose;
    /** Spot whose RPM and Kp the flywheels use, or {@code null} to use the table. */
    private ShotSpot shotSpot;
    private boolean servoAtPositionB;
    /** True while {@link #twitchFlower()} was running on the last loop. */
    private boolean flowerTwitching;
    /** Time since the current flower twitch started. */
    private final ElapsedTime flowerTwitchTimer = new ElapsedTime();
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
        // default start pose. The turret aims from this pose all match.
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

        // Two-position servo, toggled with Gamepad 1 Cross. Start at POSITION_A.
        // Skipped entirely while FLOWER_SERVO_ENABLED is false.
        servoAtPositionB = false;
        if (FLOWER_SERVO_ENABLED) {
            servo = hardwareMap.get(Servo.class, RobotConstants.FlowerRamp.NAME);
            servo.setPosition(RobotConstants.FlowerRamp.POSITION_A);
        } else {
            servo = null;
        }

        // Turret aims continuously at the hive using the follower's pose, and
        // picks the top or bottom hive side from the robot's Y position. The
        // constructor zeroes the through-bore encoder here. That zero is only
        // kept when no autonomous has just run (see below); then the turret
        // must be pointing straight forward right now (or press Gamepad 2 Triangle on
        // the init screen once it is). Must be built AFTER the follower and the
        // intake, because the encoder is read through one of their motor ports
        // (Turret.ENCODER_PORT). Skipped entirely while TURRET_ENABLED is false.
        turretZeroKeptFrom = null;
        if (TURRET_ENABLED) {
            turret = new TurretSubsystem(hardwareMap);
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
            rememberEncoderCounts();
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

        drivingToPose = false;
        shotSpot = null;
        aligning = false;
        turretHeldByAlign = false;
        speedMultiplier = RobotConstants.Drive.DEFAULT_SPEED_MULTIPLIER;

        writeInitTelemetry();
    }

    /**
     * Runs repeatedly between Init and Play. Nothing moves here: the turret
     * servos are still unpowered, so the turret can be turned by hand.
     *
     * <p>Shows the start pose the turret will aim from and the LIVE turret
     * angle and encoder ticks, so the encoder can be checked before the servos
     * get power. No buttons do anything here: the start pose and the turret
     * zero come from the autonomous that just ran, or (TeleOp started on its
     * own) the default start pose and the turret zeroed where it points at Init.
     */
    @Override
    public void init_loop() {
        if (turret != null) {
            // Read-only: updates the angle and ticks without powering the servos.
            turret.refresh();
        }

        writeInitTelemetry();
    }

    /**
     * Records the encoder count of every configured motor port, so the init screen can
     * show which counts change afterwards. Read-only; nothing is commanded.
     */
    private void rememberEncoderCounts() {
        encoderPorts.clear();
        encoderPortStartTicks.clear();
        for (DcMotorEx motor : hardwareMap.getAll(DcMotorEx.class)) {
            Set<String> names = hardwareMap.getNamesOf(motor);
            if (names.isEmpty()) {
                continue;
            }
            String name = names.iterator().next();
            encoderPorts.put(name, motor);
            encoderPortStartTicks.put(name, motor.getCurrentPosition());
        }
    }

    /**
     * Init-screen check that {@link RobotConstants.Turret#ENCODER_PORT} names the port
     * the turret encoder is really plugged into. Nothing is powered in Init, so the only
     * count that changes when the turret is turned by hand is the turret encoder's.
     */
    private void addEncoderPortCheck() {
        String turretPort = RobotConstants.Turret.ENCODER_PORT;
        StringBuilder moving = new StringBuilder();
        boolean turretPortMoving = false;
        for (Map.Entry<String, DcMotorEx> port : encoderPorts.entrySet()) {
            int moved = port.getValue().getCurrentPosition() - encoderPortStartTicks.get(port.getKey());
            if (Math.abs(moved) >= RobotConstants.Turret.ENCODER_STALL_MIN_TICKS) {
                if (moving.length() > 0) {
                    moving.append(", ");
                }
                moving.append(port.getKey()).append(" (").append(moved).append(")");
                if (port.getKey().equals(turretPort)) {
                    turretPortMoving = true;
                }
            }
        }
        telemetry.addData("Turret encoder is read from port", "'%s'", turretPort);
        telemetry.addData("Ports whose count changed since Init",
                moving.length() > 0 ? moving.toString() : "none yet (turn the turret by hand)");
        if (turretPortMoving) {
            telemetry.addLine("  OK: the turret encoder is counting on '" + turretPort + "'.");
        } else if (moving.length() > 0) {
            telemetry.addLine("  !! WRONG PORT: you turned the turret and '" + turretPort + "' did not count."
                    + " Set Turret.ENCODER_PORT in RobotConstants to the port listed above,"
                    + " or the turret will not aim.");
        } else {
            telemetry.addLine("  Check 0: turn the turret by hand. '" + turretPort + "' must show up above."
                    + " If nothing shows up, the encoder cable is not in any motor port's encoder socket.");
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
        if (turret != null) {
            addEncoderPortCheck();
            telemetry.addData("Turret measuredAngleDeg (live)", "%.1f", turret.getCurrentAngleDegrees());
            telemetry.addData("Turret ticks since zero (live)", turret.getTicksSinceZero());
            telemetry.addLine(turretZeroKeptFrom != null
                    ? "  Turret zero KEPT from " + turretZeroKeptFrom + ": a non-zero angle is correct."
                    : "  Turret zero taken at Init: it must have been pointing straight forward then.");
            telemetry.addLine("  Check 1: turn the TURRET LEFT by hand (counterclockwise from above), angle must go UP."
                    + " If it goes DOWN, flip Turret.ENCODER_REVERSED.");
            telemetry.addLine("  Check 2: turn the TURRET ONE FULL TURN by hand, angle must read 360."
                    + " If not, set Turret.ENCODER_TO_TURRET_RATIO to 8192 / (ticks since zero).");
            telemetry.addLine("  Check 3: turn the ROBOT LEFT by hand, Heading (deg) after Play must go UP.");
            telemetry.addLine("  The turret starts moving by itself when you press Play. Hands clear.");
        } else {
            telemetry.addLine("Turret: DISABLED (TURRET_ENABLED = false)");
        }
        telemetry.addLine("G1: LX/LY drive, RX turn (robot-centric)");
        telemetry.addLine("G1: Dpad Up/Down = speed +/-  |  L1 hold = slow");
        telemetry.addLine("G1: Dpad Left = drive to top angle shot  |  Dpad Right = bottom angle shot");
        telemetry.addLine("G1: Square = drive to bottom shooting spot  |  Circle = drive to opposite pose  |  any stick cancels"
                + (FLOWER_SERVO_ENABLED ? "  |  Cross = servo A/B  |  R2 hold = twitch flower" : ""));
        telemetry.addLine("G1: Triangle = align chassis to the hive on/off  |  turn stick cancels");
        telemetry.addLine("G2: Cross hold = intake  |  Triangle hold = reverse intake");
        telemetry.addLine("G2: Square hold = transfer  |  Circle hold = reverse transfer");
        telemetry.addLine("G2: L2 hold = open gate  |  R2 press = shoot balls (waits for the turret to aim)");
        telemetry.addLine("Outtake flywheels run the whole time after Play");
        if (turret != null) {
            telemetry.addLine("Turret tracks the HIVE from Play on (top/bottom side picked from the robot's Y)");
            telemetry.addLine("G2: Dpad Down = TRACK the hive");
            telemetry.addLine("G2: Dpad Up = turret faces STRAIGHT AHEAD on the robot");
            telemetry.addLine("G2: Dpad Left/Right HOLD = move turret left/right by hand, stops where released"
                    + " (Dpad LEFT must turn it LEFT and make the angle go UP)");
        }
        telemetry.addData("Gamepad 2", gamepad2.getGamepadId() == Gamepad.ID_UNASSOCIATED
                ? "NOT CONNECTED (press Options+Circle on it)" : "connected");
        telemetry.update();
    }

    /**
     * Resets the loop timer when the driver presses Play so the time spent
     * sitting in init does not show up on the first loop.
     *
     * <p>The turret is NOT re-zeroed here. Its zero came from the autonomous
     * that just ran, or was taken at Init. The encoder has been counting ever
     * since, so a turret that moved in between is still measured correctly.
     */
    @Override
    public void start() {
        lastLoopNanos = System.nanoTime();
        // The autonomous pose has now been used. Clear it so a later practice
        // TeleOp does not silently start from an old autonomous end pose.
        blackboard.remove(RobotBlackboard.LAST_AUTO_POSE);
        blackboard.remove(RobotBlackboard.LAST_AUTO_TURRET_ZERO_TICKS);
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
        if (drivingToPose && driverIsMoving(forward, strafe, turn)) {
            drivingToPose = false;
        }

        // ---- Chassis align to the hive (Gamepad 1 Triangle toggles) ----
        // The left stick still drives while aligning; the turn stick hands control back.
        // Checked here, before speed scaling, like the cancel above.
        if (alignToggle.update(gamepad1.triangle)) {
            if (aligning) {
                stopAlign();
            } else {
                startAlign();
            }
        } else if (aligning
                && Math.abs(turn) > RobotConstants.Drive.AUTOMATION_CANCEL_DEADZONE) {
            stopAlign();
        }

        boolean slowMode = gamepad1.left_bumper;
        double activeSpeed = slowMode ? RobotConstants.Drive.SLOW_MODE_MULTIPLIER : speedMultiplier;
        forward *= activeSpeed;
        strafe *= activeSpeed;
        turn *= activeSpeed;

        // Share (labeled Create on a PS5 controller) resets the Pinpoint pose to the
        // requested known position.
        // Cancel automation first so it cannot immediately drive away from the reset pose.
        if (backToggle.update(gamepad1.share)) {
            stopAlign();
            drivingToPose = false;
            follower.setPose(new Pose(2.5, 69, Math.toRadians(0.0)));
            follower.manual(0.0, 0.0, 0.0);
            fieldCentricOffsetRad = 0.0;
        }

        // ---- Optional drive-to-pose (each button needs its OWN edge detector) ----
        // Each of these also switches the flywheels to that spot's RPM and Kp.
        // Gamepad 1 D-pad Left/Right: drive to the bottom/top angle-shot pose.
        if (topAngleShootToggle.update(gamepad1.dpad_left)) {
            stopAlign();
            startDriveToShot(ShotSpot.BOTTOM_ANGLE_SHOOT);
        }
        if (bottomAngleShootToggle.update(gamepad1.dpad_right)) {
            stopAlign();
            startDriveToShot(ShotSpot.TOP_ANGLE_SHOOT);
        }

        // Gamepad 1 Square: drive to the bottom shooting spot.
        if (homeToggle.update(gamepad1.square)) {
            stopAlign();
            startDriveToShot(ShotSpot.BOTTOM_SHOOT);
        }

        // Gamepad 1 Circle: drive to the opposite start pose.
        if (opposeToggle.update(gamepad1.circle)) {
            stopAlign();
            startDriveToShot(ShotSpot.OPPOSE_POSE);
        }

        // ---- Drive ----
        if (drivingToPose) {
            if (!follower.isBusy()) {
                drivingToPose = false;
                follower.manual(0.0, 0.0, 0.0);
            }
        } else {
            if (aligning) {
                // Not scaled by slow mode: a scaled-down turn could be too weak to move.
                turn = chassisAligner.turnPower(follower.pose());
            }
            commandManualDrive(forward, strafe, turn);
        }

        // ---- Flower ramp servo (Gamepad 1 Cross toggles, R2 hold twitches) ----
        if (servo != null) {
            if (servoToggle.update(gamepad1.cross)) {
                servoAtPositionB = !servoAtPositionB;
            }
            if (gamepad1.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE) {
                twitchFlower();
            } else {
                flowerTwitching = false;
                // Sent every loop so a position edited in Panels moves the servo right away.
                servo.setPosition(servoAtPositionB
                        ? RobotConstants.FlowerRamp.POSITION_B
                        : RobotConstants.FlowerRamp.POSITION_A);
            }
        }

        // ---- Shoot balls (Gamepad 2 R2) ----
        // Ivy sequence built in shootBalls() below (it also waits for the turret
        // to aim; SecondAuto has its own version). Rising edge starts it; it runs
        // to completion. Face buttons and L2 are ignored while it is active.
        boolean shootPressed = gamepad2.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE;
        boolean shooting = isShootActive();
        if (shootPressed && !wasShootTrigger && !shooting) {
            activeShoot = shootBalls();
            schedule(activeShoot);
            shooting = true;
        }
        wasShootTrigger = shootPressed;

        // ---- Intake / outtake ----
        // Flywheels stay on for the whole TeleOp, including during shoot. Their RPM
        // follows the distance to the hive (see ShooterRPMTable), except at a shot spot
        // picked with a drive-to-pose button, which has its own RPM and Kp.
        if (shotSpot != null && !drivingToPose
                && follower.pose().distance(shotSpot.pose)
                > RobotConstants.ShotSpots.RELEASE_DISTANCE_INCHES) {
            shotSpot = null;
        }
        if (shotSpot != null && !ShooterRPMTable.TUNING_MODE) {
            outtake.outtakeAt(shotSpot.rpm.getAsDouble(), shotSpot.kp.getAsDouble());
        } else {
            outtake.outtakeFrom(follower.pose());
        }
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
     * Twitches the flower servo between positions B and C, switching every
     * {@link RobotConstants.FlowerRamp#TWITCH_INTERVAL_MS}. Call it every loop for as long
     * as the servo should twitch; the first call starts with a move to C. Does not block.
     */
    private void twitchFlower() {
        if (!flowerTwitching) {
            flowerTwitching = true;
            flowerTwitchTimer.reset();
        }
        long step = (long) (flowerTwitchTimer.milliseconds()
                / RobotConstants.FlowerRamp.TWITCH_INTERVAL_MS);
        servo.setPosition(step % 2 == 0
                ? RobotConstants.FlowerRamp.POSITION_C
                : RobotConstants.FlowerRamp.POSITION_B);
    }

    /**
     * Starts a straight line from the robot's current pose to the given target.
     *
     * <p>If the robot is already there, a zero-length line path cannot be
     * constructed, so it simply holds that pose instead.
     */
    private void startDriveToPose(Pose target) {
        Pose current = follower.pose();

        if (current.distance(target) < 0.5) {
            follower.hold(target);
            drivingToPose = false;
            return;
        }

        Path targetPath = line(current, target).linear(current, target);
        follower.follow(targetPath);
        drivingToPose = true;
    }

    /** Drives to a shot spot and switches the flywheels to its RPM and Kp. */
    private void startDriveToShot(ShotSpot spot) {
        shotSpot = spot;
        startDriveToPose(spot.pose);
    }

    /**
     * Gamepad 1 Triangle: starts turning the chassis toward the hive side on the robot's half of
     * the field. Ends any drive-to-pose. With
     * {@link RobotConstants.ChassisAlign#HOLD_TURRET_STRAIGHT} the turret is also held
     * straight ahead until {@link #stopAlign()}.
     */
    private void startAlign() {
        aligning = true;
        drivingToPose = false;
        boolean top = follower.pose().y() >= RobotConstants.Turret.SIDE_SWITCH_Y_INCHES;
        if (turret != null) {
            if (RobotConstants.ChassisAlign.HOLD_TURRET_STRAIGHT) {
                turretModeBeforeAlign = turret.getMode();
                turretHeldByAlign = true;
                // A jog released just now must not switch the turret back to MANUAL.
                turretJogging = false;
                turret.setHomeAngle(0.0);
                turret.setMode(TurretSubsystem.TurretMode.HOME);
            } else if (turret.getMode() == TurretSubsystem.TurretMode.TRACKING) {
                // Both are aiming, so pick the same side, also near the switch line.
                top = turret.isAimingAtTopSide();
            }
        }
        chassisAligner.start(top);
    }

    /**
     * Gives turning back to the driver, and returns the turret to what it was doing unless
     * the operator has taken it over since. Does nothing if not aligning.
     */
    private void stopAlign() {
        if (!aligning) {
            return;
        }
        aligning = false;
        if (turretHeldByAlign) {
            turretHeldByAlign = false;
            turret.setHomeAngle(Math.toRadians(RobotConstants.Turret.HOME_ANGLE_DEGREES));
            turret.setMode(turretModeBeforeAlign);
        }
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
     * Gamepad 2 face buttons: Cross hold = intake, Triangle hold = reverse intake,
     * Square hold = transfer, Circle hold = reverse transfer.
     */
    private void updateIntakeFromGamepad() {
        if (gamepad2.triangle) {
            intake.reverse();
        } else if (gamepad2.cross) {
            intake.intake();
        } else {
            intake.stopIntake();
        }

        if (gamepad2.square) {
            intake.transfer();
        } else if (gamepad2.circle) {
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
     *         there is no turret, the turret is aimed at its target, the
     *         operator has deliberately parked it (HOME / MANUAL / DISABLED) and
     *         is aiming with the whole robot instead, or a fault has stopped it
     *         (it will not get any closer by waiting)
     */
    private boolean turretReadyToShoot() {
        return turret == null
                || turret.getMode() != TurretSubsystem.TurretMode.TRACKING
                || turret.hasEncoderFault()
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
     * <ul>
     *   <li>Dpad Left (hold): move the turret LEFT (counterclockwise from above) at
     *       {@link RobotConstants.Turret#JOG_POWER}. It stops where it is when the button
     *       is released and stays in MANUAL until Dpad Up or Down is pressed.</li>
     *   <li>Dpad Right (hold): the same, to the RIGHT.</li>
     *   <li>Dpad Up: face STRAIGHT AHEAD on the robot and hold that
     *       ({@link RobotConstants.Turret#HOME_ANGLE_DEGREES}). Pressing it again changes
     *       nothing.</li>
     *   <li>Dpad Down: TRACK the hive. The aim comes from the turret encoder angle and
     *       the hive position on the field, and the robot's field Y picks the top or
     *       bottom hive side.</li>
     * </ul>
     *
     * <p>Dpad Up and Down also clear a turret fault stop and try again. While a jog button
     * is held it wins over Up / Down; press Up or Down after letting go.
     *
     * <p>The turret is in TRACKING from Play on (set in {@link #init()}).
     */
    private void updateTurretFromGamepad() {
        // Read both edges every loop, even while jogging, so a button held through a jog
        // does not fire later by itself.
        boolean forwardPressed = turretForwardToggle.update(gamepad2.dpad_up);
        boolean hivePressed = turretHiveToggle.update(gamepad2.dpad_down);

        // The operator's turret buttons win over chassis align: from now on it must not
        // hand the turret back to its old mode.
        if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) {
            turretHeldByAlign = false;
        }

        // Manual move: held buttons, not edges. Left = +power (counterclockwise).
        boolean jogLeft = gamepad2.dpad_left;
        boolean jogRight = gamepad2.dpad_right;
        if (jogLeft != jogRight) {
            turret.jog(jogLeft ? RobotConstants.Turret.JOG_POWER : -RobotConstants.Turret.JOG_POWER);
            turretJogging = true;
            return;
        }
        if (turretJogging) {
            // Both released (or both held): stop where it is. Stays in MANUAL.
            turret.jog(0.0);
            turretJogging = false;
        }

        if (hivePressed) {
            turret.clearFault();
            turret.setAutoSelectSide(true);
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        } else if (forwardPressed) {
            turret.clearFault();
            turret.setHomeAngle(Math.toRadians(RobotConstants.Turret.HOME_ANGLE_DEGREES));
            turret.setMode(TurretSubsystem.TurretMode.HOME);
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
        // Turret first: its state, warnings and angles must be visible without
        // scrolling the Driver Station, because they show why it is or is not aiming.
        if (turret != null) {
            turret.addTelemetry(telemetry);
            telemetry.addData("Turret target (in)", "(%.1f, %.1f), %.1f in away",
                    turret.getTargetX(), turret.getTargetY(), turret.getDistanceToTarget(pose));
        } else {
            telemetry.addData("Turret", "DISABLED (TURRET_ENABLED = false)");
        }
        if (aligning) {
            telemetry.addData("Chassis align", "%s hive, error %.1f deg%s",
                    chassisAligner.isAimingAtTopSide() ? "TOP" : "BOTTOM",
                    chassisAligner.getErrorDegrees(),
                    chassisAligner.isAligned() ? " (ALIGNED)" : "");
        } else {
            telemetry.addData("Chassis align", "OFF (G1 Triangle)");
        }
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Start pose was", startPoseSource);
        telemetry.addData("Drive mode", fieldCentric ? "FIELD" : "ROBOT");
        telemetry.addData("Drive speed", "%.0f%%", speedMultiplier * 100.0);
        telemetry.addData("Slow mode", slowMode ? "ON" : "OFF");
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Transfer", intake.getTransferState());
        if (shotSpot != null && !ShooterRPMTable.TUNING_MODE) {
            telemetry.addData("Shooter spot", "%s: %.0f RPM, Kp L %.5f / R %.5f",
                    shotSpot, outtake.getTargetRpm(), outtake.getLeftKp(), outtake.getRightKp());
        } else {
            ShooterRPMTable.addTelemetry(telemetry, pose);
        }
        telemetry.addData("Outtake Motor", outtake.getMotorState());
        telemetry.addData("Outtake RPM (L/R)", "%.0f / %.0f (target=%.0f, atSpeed=%b)",
                outtake.getLeftRpm(), outtake.getRightRpm(), outtake.getTargetRpm(), outtake.atSpeed());
        telemetry.addData("Outtake Vbat / command (L/R)", "%.2f V / %.2f V, %.2f V",
                outtake.getBatteryVoltage(), outtake.getLeftVoltageCommand(),
                outtake.getRightVoltageCommand());
        if (!outtake.getControllerFault().isEmpty()) {
            telemetry.addData("Outtake controller fault", outtake.getControllerFault());
        }
        telemetry.addData("Outtake Gate", outtake.isGateOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Shoot balls", isShootActive() ? "RUNNING" : "OFF");
        telemetry.addData("Servo", servo == null
                ? "DISABLED"
                : flowerTwitching
                ? "TWITCHING"
                : (servoAtPositionB ? "POSITION_B" : "POSITION_A"));
        telemetry.addData("Gamepad 2", gamepad2.getGamepadId() == Gamepad.ID_UNASSOCIATED
                ? "NOT CONNECTED (press Options+Circle on it)" : "connected");
        telemetry.addData("Drive to pose", drivingToPose ? "FOLLOWING" : "OFF");
        telemetry.addData("Loop (ms)", loopMs);
        telemetry.update();
    }
}
