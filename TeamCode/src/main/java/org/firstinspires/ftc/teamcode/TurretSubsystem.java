package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric turret driven by TWO continuous-rotation servos, continuously
 * aiming at a field coordinate (the hive) using the Pedro Pathing pose.
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>Two {@link CRServo}s ({@code SERVO_LEFT}, {@code SERVO_RIGHT}) that BOTH push the
 *       turret in the same direction (set the reversed flags in RobotConstants).</li>
 *   <li>A REV Through Bore Encoder V2 geared to the turret. Its cable is plugged
 *       into the encoder socket of an existing motor port ({@code ENCODER_PORT}), and the
 *       count is read through that motor. This class only READS that motor's encoder; it
 *       never changes the motor's mode, power or direction, and the motor's own
 *       direction setting does not affect the turret angle.</li>
 * </ul>
 *
 * <h3>The encoder is the only feedback</h3>
 * HOME and TRACKING aim with the measured angle. If {@code ENCODER_PORT} names the wrong
 * motor port (or the cable is out) the measured angle never changes, the error never
 * shrinks, and the servos would turn the turret in circles forever. This class therefore
 * watches the count whenever it drives the servos and stops them with a message on the
 * Driver Station when the count does not follow. MANUAL jog is open-loop and keeps
 * working without the encoder. A single reading that jumps farther than the turret could
 * possibly have turned is ignored (see
 * {@link RobotConstants.Turret#ENCODER_MAX_JUMP_DEGREES}).
 *
 * <h3>Hybrid controller</h3>
 * The servos are continuous-rotation, so power sets a SPEED, not a position: the encoder
 * closes the loop on every update. {@code calculateHybridPower} takes the direction from
 * the sign of the angle error and the amount of power from its size:
 * <ul>
 *   <li><b>Far</b> (beyond {@link RobotConstants.Turret#FAR_ERROR_DEGREES}):
 *       {@link RobotConstants.Turret#FAR_POWER}.</li>
 *   <li><b>Slowing down</b>: a continuous curve from FAR_POWER to
 *       {@link RobotConstants.Turret#FINE_POWER}.</li>
 *   <li><b>Fine</b> (inside {@link RobotConstants.Turret#FINE_ERROR_DEGREES}): from
 *       FINE_POWER down to {@link RobotConstants.Turret#MIN_COMMAND_POWER}.</li>
 *   <li><b>On target</b>: no power, unless the target is moving.</li>
 * </ul>
 * While the target is moving, velocity feedforward is added so the turret keeps up with it.
 * There is no integral or derivative term.
 *
 * <h3>Settling (no jitter at the target)</h3>
 * The turret has a minimum correction power (a servo cannot move at less than its dead
 * zone), so a correction is always a fixed-size kick no matter how small the error is.
 * With a single tolerance that kick can carry the turret across the whole tolerance band
 * and the turret then kicks back, again and again. The aimed state therefore has
 * HYSTERESIS: the turret stops once it is inside
 * {@link RobotConstants.Turret#ANGLE_TOLERANCE_DEGREES} and stays stopped until the error
 * grows beyond {@link RobotConstants.Turret#ANGLE_EXIT_TOLERANCE_DEGREES}. The wider exit
 * band applies only while the target is NOT moving: while the chassis is turning or driving
 * past the hive the tight tolerance applies in both directions, so tracking stays as tight
 * as before. The "target is moving" test has hysteresis too, so a target speed that hovers
 * around the threshold does not toggle between feedforward and no power every few loops.
 *
 * <p>The hysteresis cannot help if ONE correction kick (the speed at
 * {@link RobotConstants.Turret#MIN_COMMAND_POWER} times the loop period, plus coast) is
 * larger than the exit band: the turret then still jumps from one side of the band to the
 * other. In that case make the kick smaller (lower MIN_COMMAND_POWER, shorter loop) or widen
 * the exit tolerance.
 *
 * <h3>Zero</h3>
 * The encoder is relative, so "0" is wherever the turret points when this object is
 * created (OpMode INIT) or when {@link #resetAngle()} is called, and the turret must be
 * pointing straight forward at that moment, UNLESS the OpMode then calls
 * {@link #restoreZero(int)} with a zero saved earlier (Main TeleOp does this straight
 * after an autonomous). After that the angle is MEASURED every loop, so moving the
 * turret by hand or bumping it does not lose the zero.
 *
 * <p>The servos are not commanded until the first {@link #update(Pose)}, so they stay
 * unpowered during INIT and the turret can be turned by hand to check the encoder.
 *
 * <h3>Live tuning from Panels</h3>
 * Every tunable value in {@link RobotConstants.Turret} is a non-final static field, and
 * {@code RobotConstants.Turret} is {@code @Configurable}, so Panels can change it while an
 * OpMode runs. Most values are read from there on every loop. The few this class keeps a
 * working copy of (feedforward gains, pivot offset, soft limits, home angle and the
 * direction flags) are picked up by {@code applyLiveConfig()} at the start of
 * {@link #update(Pose)} and {@link #refresh()}.
 */
public class TurretSubsystem {

    /** Slack on the soft-limit comparison, so a target ON a limit is not read as past it. */
    private static final double LIMIT_EPSILON_RAD = 1e-9;

    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx turretEncoder;
    private final ElapsedTime targetVelTimer = new ElapsedTime();

    /** High-level turret operating mode. */
    public enum TurretMode {
        /** Continuously aims at the field target coordinate. */
        TRACKING,
        /**
         * Turns to and holds a fixed chassis-relative angle
         * ({@link RobotConstants.Turret#HOME_ANGLE_DEGREES} unless changed with
         * {@link #setHomeAngle(double)}).
         */
        HOME,
        /**
         * Operator control ({@link #jog(double)}): the servos get a fixed power while a
         * jog button is held and zero power otherwise. Open-loop, so it works even when
         * the encoder does not.
         */
        MANUAL,
        /** Servos commanded to zero power (stopped). */
        DISABLED
    }

    /** Power applied while in {@link TurretMode#MANUAL} (+ = counterclockwise). */
    private double manualPower = 0.0;

    // ---- Direction / encoder monitor ----
    // Runtime servo sign (+1 or -1). Flipped when the encoder moves AGAINST a sustained
    // commanded power: the servo and encoder directions disagree. Keeps a wrong flag
    // from turning the control loop into a runaway.
    private int servoSign = 1;
    private int autoFlipCount = 0;
    /** True once HOME/TRACKING was stopped by the monitor (see {@link #directionFault}). */
    private boolean encoderFault = false;
    /** With {@link #encoderFault}: true = direction kept flipping, false = count did not move. */
    private boolean directionFault = false;
    /** True if a manual jog moved the measured angle the wrong way (reported, not corrected). */
    private boolean jogDirectionMismatch = false;
    /** True once the encoder count has changed since this object was built. */
    private boolean encoderSeenMoving = false;
    private int ticksAtConstruction = 0;

    // Direction window: where the turret was when the command last changed direction.
    private double directionPowerSign = 0.0;
    private double directionStartAngleRad = 0.0;
    private double directionAllowedRad = 0.0;
    // Command reversals seen far from the target (turret buzzing opposite the target).
    private int farReversalCount = 0;
    private final ElapsedTime farReversalTimer = new ElapsedTime();
    // Stall window: the encoder count when the servos were last seen making progress.
    private double stallPowerSign = 0.0;
    private int stallStartTicks = 0;
    private final ElapsedTime stallTimer = new ElapsedTime();
    // Measured turret speed, for the direction window's coast allowance.
    private double previousMonitorAngleRad = 0.0;
    private final ElapsedTime monitorLoopTimer = new ElapsedTime();

    private TurretMode mode = TurretMode.TRACKING;

    /** Chassis-relative angle (radians) held while in {@link TurretMode#HOME}. */
    private double homeAngleRad = Math.toRadians(RobotConstants.Turret.HOME_ANGLE_DEGREES);

    // Geometry
    private double xOffsetInches = RobotConstants.Turret.OFFSET_X_INCHES;
    private double yOffsetInches = RobotConstants.Turret.OFFSET_Y_INCHES;

    // Encoder zero. Stored as an offset instead of resetting the hub encoder, because the
    // encoder shares its port with a real motor and because this makes resetAngle() cheap
    // and safe to call at any time.
    private int zeroOffsetTicks = 0;
    private int lastEncoderTicks = 0;

    // Rotation soft limits (prevent wire binding)
    private boolean useSoftLimits = RobotConstants.Turret.USE_SOFT_LIMITS;
    private double minAngleRadians = Math.toRadians(RobotConstants.Turret.MIN_ANGLE_DEGREES);
    private double maxAngleRadians = Math.toRadians(RobotConstants.Turret.MAX_ANGLE_DEGREES);

    // Field target (hive) in Pedro Pathing inches. Until the first pose arrives it is the
    // bottom side; selectTargetSide() then reads the live values every loop.
    private double targetXInches = RobotConstants.Turret.BOTTOM_TARGET_X_INCHES;
    private double targetYInches = RobotConstants.Turret.BOTTOM_TARGET_Y_INCHES;

    // The hive has two sides. By default the side is picked from the robot's Y position.
    private boolean autoSelectSide = true;
    private boolean aimingAtTopSide = false;
    /** False until the side has been picked from a real pose (then hysteresis applies). */
    private boolean sideChosen = false;

    // Feedforward: power per rad/s of target motion, and the breakaway power.
    private double kV = RobotConstants.Turret.KV;
    private double kF = RobotConstants.Turret.KF;
    /** Rate-limited velocity feedforward power applied on the last update. */
    private double feedforwardPower = 0.0;
    /** True if the last encoder reading was ignored as impossible. */
    private boolean readingRejected = false;

    // Internal tracking state
    private double previousTargetAngleRad = 0.0;
    private double targetAngularVelocityRadSec = 0.0; // low-pass filtered
    private double rawTargetVelocityRadSec = 0.0;     // unfiltered, this loop only
    private boolean firstTargetSample = true;
    /**
     * Latched "target is moving" flag, with hysteresis: it turns on above
     * {@link RobotConstants.Turret#TARGET_MOVING_THRESHOLD_DEG_SEC} and only turns off
     * again below half of that. A filtered target speed that hovers around a single
     * threshold would otherwise switch the on-target output between zero power and
     * feedforward every few loops.
     */
    private boolean targetMovingLatched = false;
    private boolean enabled = true;

    private double currentAngleRad = 0.0;
    private double targetGlobalAngleRad = 0.0;
    private double relativeTargetAngleRad = 0.0;
    private double angleErrorRad = 0.0;
    private double lastCommandedPower = 0.0;

    /**
     * True while the turret counts as aimed. Has hysteresis: it turns on inside
     * {@link RobotConstants.Turret#ANGLE_TOLERANCE_DEGREES} and, while the target is not
     * moving, stays on until the error exceeds
     * {@link RobotConstants.Turret#ANGLE_EXIT_TOLERANCE_DEGREES}.
     */
    private boolean onTarget = false;
    /** True if the last commanded angle had to be clipped to a soft limit. */
    private boolean targetClipped = false;

    // ---- Live tuning (Panels) ----
    // The RobotConstants.Turret values are non-final so Panels can edit them while an OpMode
    // runs. Most are read straight from there every loop. The few this class keeps a working
    // copy of are re-applied by applyLiveConfig(), but only when the configured value
    // CHANGED since the last look, so the setters (setGains, setMountingOffset, ...) keep
    // working until somebody edits the matching value in Panels.
    private double appliedKf = RobotConstants.Turret.KF;
    private double appliedKv = RobotConstants.Turret.KV;
    private double appliedOffsetX = RobotConstants.Turret.OFFSET_X_INCHES;
    private double appliedOffsetY = RobotConstants.Turret.OFFSET_Y_INCHES;
    private boolean appliedUseSoftLimits = RobotConstants.Turret.USE_SOFT_LIMITS;
    private double appliedMinAngleDeg = RobotConstants.Turret.MIN_ANGLE_DEGREES;
    private double appliedMaxAngleDeg = RobotConstants.Turret.MAX_ANGLE_DEGREES;
    private double appliedHomeAngleDeg = RobotConstants.Turret.HOME_ANGLE_DEGREES;
    private boolean appliedLeftReversed = RobotConstants.Turret.SERVO_LEFT_REVERSED;
    private boolean appliedRightReversed = RobotConstants.Turret.SERVO_RIGHT_REVERSED;
    private boolean appliedEncoderReversed = RobotConstants.Turret.ENCODER_REVERSED;

    /**
     * Looks up both continuous servos and the encoder, and zeroes the angle where the
     * turret points now. That zero is only right if the turret is straight forward;
     * otherwise call {@link #restoreZero(int)} or {@link #resetAngle()} afterwards.
     *
     * <p>Nothing is commanded here: the servos stay unpowered until the first
     * {@link #update(Pose)}, and the motor that shares the encoder port is left exactly
     * as its own subsystem configured it.
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, RobotConstants.Turret.SERVO_LEFT);
        rightServo = hardwareMap.get(CRServo.class, RobotConstants.Turret.SERVO_RIGHT);

        // Both servos must push the turret the SAME way, otherwise they fight each other.
        leftServo.setDirection(RobotConstants.Turret.SERVO_LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightServo.setDirection(RobotConstants.Turret.SERVO_RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // The encoder cable is in the encoder socket of an existing motor port, so the SDK
        // exposes the count through that motor. Read-only: no setMode, setPower or
        // setDirection here, or the real motor on that port would be disturbed.
        turretEncoder = hardwareMap.get(DcMotorEx.class, RobotConstants.Turret.ENCODER_PORT);
        resetAngle();
        ticksAtConstruction = lastEncoderTicks;

        targetVelTimer.reset();
    }

    /**
     * Declares the turret's current position to be 0 rad (pointing forward).
     * Call only when it really is. Stores an offset instead of resetting the
     * hub encoder, so this is safe to call at any time.
     */
    public void resetAngle() {
        zeroOffsetTicks = readRawTicks();
        lastEncoderTicks = zeroOffsetTicks;
        currentAngleRad = 0.0;
        // The aimed state belongs to the old zero: let the next update() decide it again.
        onTarget = false;
        restartMonitor();
    }

    /** @return encoder counts the turret has moved from its zero (unsigned by ENCODER_REVERSED) */
    public int getTicksSinceZero() {
        return lastEncoderTicks - zeroOffsetTicks;
    }

    /**
     * @return true once the encoder count has changed since this object was built. While
     *         this is false after the turret was turned (by hand or by the servos), the
     *         encoder is not being read: check {@link RobotConstants.Turret#ENCODER_PORT}.
     */
    public boolean isEncoderSeenMoving() {
        return encoderSeenMoving;
    }

    /** @return raw encoder count that currently means 0 rad (turret straight forward) */
    public int getZeroOffsetTicks() {
        return zeroOffsetTicks;
    }

    /**
     * Restores a zero saved earlier with {@link #getZeroOffsetTicks()}, instead of
     * declaring the current position to be 0. The hub keeps counting between OpModes, so
     * a saved zero stays valid as long as the hub has not lost power since.
     *
     * @param zeroTicks raw encoder count that means turret straight forward
     */
    public void restoreZero(int zeroTicks) {
        zeroOffsetTicks = zeroTicks;
        currentAngleRad = readAngleRadians();
        // The aimed state belongs to the old zero: let the next update() decide it again.
        onTarget = false;
        restartMonitor();
    }

    /**
     * Hub encoder count with the shared motor's direction setting undone. The SDK negates
     * the count when a motor is set to REVERSE; that setting belongs to the motor that
     * shares the port (the intake), so flipping the intake's direction must never flip the
     * turret angle. Read-only. (The motor TYPE chosen in the config can still flip the
     * sign; {@link RobotConstants.Turret#ENCODER_REVERSED} covers that.)
     */
    private int readRawTicks() {
        return rawTicks(turretEncoder);
    }

    private static int rawTicks(DcMotorEx encoderMotor) {
        int ticks = encoderMotor.getCurrentPosition();
        return encoderMotor.getDirection() == DcMotorSimple.Direction.REVERSE ? -ticks : ticks;
    }

    /**
     * Reads the turret encoder's raw count WITHOUT building a turret: no servo is looked up
     * and nothing is commanded. Autonomous uses this to record the count that means
     * "turret straight forward", so TeleOp can restore it with {@link #restoreZero(int)}
     * and know the turret's real angle even if it moved during autonomous.
     *
     * @return raw encoder count, in the same units as {@link #getZeroOffsetTicks()}
     */
    public static int readEncoderTicks(HardwareMap hardwareMap) {
        return rawTicks(hardwareMap.get(DcMotorEx.class, RobotConstants.Turret.ENCODER_PORT));
    }

    /**
     * Reads the encoder without commanding the servos. Call from {@code init_loop()} so
     * the angle and tick telemetry are live before Play while the servos are unpowered.
     */
    public void refresh() {
        applyLiveConfig();
        currentAngleRad = readAngleRadians();
    }

    /**
     * Picks up values edited in Panels since the last call. Runs at the start of
     * {@link #update(Pose)} and {@link #refresh()}. Only values that CHANGED are applied, so
     * the setters ({@link #setGains}, {@link #setMountingOffset}, ...) keep working until the
     * matching value is edited in Panels. Everything not handled here is read straight from
     * {@link RobotConstants.Turret} wherever it is used.
     */
    private void applyLiveConfig() {
        double kf = RobotConstants.Turret.KF;
        double kv = RobotConstants.Turret.KV;
        if (kf != appliedKf || kv != appliedKv) {
            this.kF = kf;
            this.kV = kv;
            appliedKf = kf;
            appliedKv = kv;
        }

        double offsetX = RobotConstants.Turret.OFFSET_X_INCHES;
        double offsetY = RobotConstants.Turret.OFFSET_Y_INCHES;
        if (offsetX != appliedOffsetX || offsetY != appliedOffsetY) {
            setMountingOffset(offsetX, offsetY);
            appliedOffsetX = offsetX;
            appliedOffsetY = offsetY;
        }

        boolean limits = RobotConstants.Turret.USE_SOFT_LIMITS;
        double minDeg = RobotConstants.Turret.MIN_ANGLE_DEGREES;
        double maxDeg = RobotConstants.Turret.MAX_ANGLE_DEGREES;
        if (limits != appliedUseSoftLimits || minDeg != appliedMinAngleDeg
                || maxDeg != appliedMaxAngleDeg) {
            setSoftLimitsDegrees(limits, minDeg, maxDeg);
            appliedUseSoftLimits = limits;
            appliedMinAngleDeg = minDeg;
            appliedMaxAngleDeg = maxDeg;
        }

        double homeDeg = RobotConstants.Turret.HOME_ANGLE_DEGREES;
        if (homeDeg != appliedHomeAngleDeg) {
            setHomeAngle(Math.toRadians(homeDeg));
            appliedHomeAngleDeg = homeDeg;
        }

        boolean leftReversed = RobotConstants.Turret.SERVO_LEFT_REVERSED;
        boolean rightReversed = RobotConstants.Turret.SERVO_RIGHT_REVERSED;
        boolean encoderReversed = RobotConstants.Turret.ENCODER_REVERSED;
        if (leftReversed != appliedLeftReversed || rightReversed != appliedRightReversed
                || encoderReversed != appliedEncoderReversed) {
            if (leftReversed != appliedLeftReversed) {
                leftServo.setDirection(leftReversed
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }
            if (rightReversed != appliedRightReversed) {
                rightServo.setDirection(rightReversed
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }
            appliedLeftReversed = leftReversed;
            appliedRightReversed = rightReversed;
            appliedEncoderReversed = encoderReversed;

            // A direction flag was edited, so forget the run-time servo flip that was made to
            // compensate for the OLD flags (otherwise the fix and the flip cancel out), and
            // clear a stop that came from the directions disagreeing. Re-read the angle first:
            // ENCODER_REVERSED changes its sign, and the monitor must start from the new one.
            servoSign = 1;
            autoFlipCount = 0;
            jogDirectionMismatch = false;
            if (directionFault) {
                encoderFault = false;
                directionFault = false;
            }
            feedforwardPower = 0.0;
            currentAngleRad = readAngleRadians();
            restartMonitor();
        }
    }

    /**
     * Reads the encoder and converts ticks to a chassis-relative turret angle.
     * The result is continuous (it keeps counting past +/- PI), which is what the
     * soft limits need; {@link #driveToRelativeAngle} wraps the ERROR, not the angle.
     */
    private double readAngleRadians() {
        int rawTicks = readRawTicks();
        lastEncoderTicks = rawTicks;
        if (Math.abs(rawTicks - ticksAtConstruction) >= RobotConstants.Turret.ENCODER_STALL_MIN_TICKS) {
            encoderSeenMoving = true;
        }
        int ticks = rawTicks - zeroOffsetTicks;
        if (RobotConstants.Turret.ENCODER_REVERSED) {
            ticks = -ticks;
        }
        return (ticks / RobotConstants.Turret.ENCODER_TICKS_PER_REV)
                * (2.0 * Math.PI)
                * RobotConstants.Turret.ENCODER_TO_TURRET_RATIO;
    }

    /**
     * Reads the encoder into {@link #currentAngleRad} for the control loop.
     *
     * <p>A change since the last loop that is larger than the turret could have turned, plus
     * {@link RobotConstants.Turret#ENCODER_MAX_JUMP_DEGREES}, is treated as a bad read and
     * the previous angle is kept. Only ONE reading in a row is ignored: if the next one
     * still says so, it is believed, so this can never lock the angle.
     *
     * @return false if no usable angle can be computed (a zero or non-numeric
     *         {@link RobotConstants.Turret#ENCODER_TO_TURRET_RATIO}); the angle is then
     *         left unchanged and HOME / TRACKING must not drive
     */
    private boolean measureAngle() {
        double reading = readAngleRadians();
        if (!isFinite(reading) || RobotConstants.Turret.ENCODER_TO_TURRET_RATIO == 0.0) {
            return false;
        }
        double margin = RobotConstants.Turret.ENCODER_MAX_JUMP_DEGREES;
        double seconds = Math.min(monitorLoopTimer.seconds(), 0.5);
        double possible = Math.toRadians(margin)
                + RobotConstants.Turret.RAD_PER_SEC_AT_FULL_POWER * seconds;
        if (margin > 0.0 && !readingRejected && Math.abs(reading - currentAngleRad) > possible) {
            readingRejected = true;
            return true;
        }
        readingRejected = false;
        currentAngleRad = reading;
        return true;
    }

    /**
     * Call once per loop. Computes the chassis-relative angle to the target from the
     * Pedro pose and drives the servos with the hybrid controller on the encoder angle.
     *
     * @param currentPose current robot {@link Pose} from Pedro Pathing
     */
    public void update(Pose currentPose) {
        applyLiveConfig();
        boolean angleValid = measureAngle();
        monitorTurret();

        if (!enabled || mode == TurretMode.DISABLED) {
            stop();
            return;
        }

        // MANUAL comes before the fault check on purpose: the jog is open-loop, so it is
        // the operator's way to move the turret when the encoder is not working.
        if (mode == TurretMode.MANUAL) {
            onTarget = false;
            double power = manualPower;
            if (useSoftLimits) {
                if ((currentAngleRad >= maxAngleRadians && power > 0.0)
                        || (currentAngleRad <= minAngleRadians && power < 0.0)) {
                    power = 0.0;
                }
            }
            setServoPower(Range.clip(power, -RobotConstants.Turret.MAX_POWER,
                    RobotConstants.Turret.MAX_POWER));
            return;
        }

        // HOME and TRACKING aim with the measured angle. Without a working measurement
        // they would only spin the turret, so they stay stopped until the fault is cleared.
        if (encoderFault || !angleValid) {
            stop();
            return;
        }

        if (mode == TurretMode.HOME) {
            driveToRelativeAngle(homeAngleRad);
            return;
        }

        if (currentPose == null) {
            stop();
            return;
        }

        selectTargetSide(currentPose);

        // 1. Robot frame -> world frame (turret pivot position)
        double robotX = currentPose.x();
        double robotY = currentPose.y();
        double robotHeading = currentPose.heading();

        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double turretWorldX = robotX + (xOffsetInches * cosH - yOffsetInches * sinH);
        double turretWorldY = robotY + (xOffsetInches * sinH + yOffsetInches * cosH);

        // 2. Global angle to hive -> chassis-relative angle in [-PI, PI]
        targetGlobalAngleRad = Math.atan2(targetYInches - turretWorldY, targetXInches - turretWorldX);
        double relativeTargetAngle = AngleUnit.normalizeRadians(targetGlobalAngleRad - robotHeading);

        driveToRelativeAngle(relativeTargetAngle);
    }

    private void driveToRelativeAngle(double relativeTargetAngle) {
        targetClipped = false;
        if (useSoftLimits) {
            double clipped = Range.clip(relativeTargetAngle, minAngleRadians, maxAngleRadians);
            targetClipped = Math.abs(clipped - relativeTargetAngle) > 1e-9;
            relativeTargetAngle = clipped;
        }
        relativeTargetAngleRad = relativeTargetAngle;

        // Target angular velocity (rad/s) for kV feedforward. It is capped at the turret's
        // top speed and low-pass filtered, so one jittery loop time cannot spike the
        // feedforward. A deliberate target jump (side switch, new target, new home angle)
        // sets firstTargetSample so the jump is not mistaken for a fast-moving target.
        double dt = targetVelTimer.seconds();
        targetVelTimer.reset();
        double maxVelocity = RobotConstants.Turret.RAD_PER_SEC_AT_FULL_POWER;
        double feedforwardDt = 0.0; // stays 0 when this interval cannot be used
        if (!firstTargetSample && dt > 1e-4 && dt <= 0.5) {
            feedforwardDt = dt;
            double rawVelocity = Range.clip(
                    AngleUnit.normalizeRadians(relativeTargetAngleRad - previousTargetAngleRad) / dt,
                    -maxVelocity, maxVelocity);
            double alpha = dt / (RobotConstants.Turret.TARGET_VELOCITY_FILTER_SECONDS + dt);
            targetAngularVelocityRadSec += alpha * (rawVelocity - targetAngularVelocityRadSec);
            rawTargetVelocityRadSec = rawVelocity;
        } else {
            firstTargetSample = false;
            targetAngularVelocityRadSec = 0.0;
            rawTargetVelocityRadSec = 0.0;
        }
        previousTargetAngleRad = relativeTargetAngleRad;

        // "Target is moving" with hysteresis: on above the threshold, off again only below
        // half of it. A filtered speed that hovers around ONE threshold would otherwise flip
        // the on-target output between zero power and feedforward every few loops.
        double targetSpeed = Math.abs(targetAngularVelocityRadSec);
        double movingEnter = Math.toRadians(RobotConstants.Turret.TARGET_MOVING_THRESHOLD_DEG_SEC);
        targetMovingLatched = targetSpeed > (targetMovingLatched ? 0.5 * movingEnter : movingEnter);
        boolean targetMoving = targetMovingLatched;

        // Shortest path from the measured angle to the target, across +/- PI and after
        // any number of full revolutions.
        angleErrorRad = AngleUnit.normalizeRadians(relativeTargetAngleRad - currentAngleRad);

        // Don't let the shortest path cross the mechanical limits (cable wrap)
        if (useSoftLimits) {
            double unwrappedTarget = currentAngleRad + angleErrorRad;
            if (unwrappedTarget > maxAngleRadians + LIMIT_EPSILON_RAD) {
                angleErrorRad -= 2.0 * Math.PI;
            } else if (unwrappedTarget < minAngleRadians - LIMIT_EPSILON_RAD) {
                angleErrorRad += 2.0 * Math.PI;
            }
        }

        // Aimed, with hysteresis. The turret becomes aimed inside the (tight) tolerance and
        // stays aimed until the error grows beyond the (wider) exit tolerance. Without this
        // a correction kick at the minimum power can carry the turret across the whole
        // tolerance band, and the kick back does the same in the other direction: jitter.
        // The wider exit band is used only while the target is NOT moving; a moving target
        // keeps the tight tolerance so tracking does not get looser while the chassis turns.
        // onTarget still holds last loop's value here; every place that must forget it
        // (mode change, new target, stop, new zero) sets it to false.
        double absError = Math.abs(angleErrorRad);
        double enterTolerance = Math.toRadians(RobotConstants.Turret.ANGLE_TOLERANCE_DEGREES);
        double exitTolerance = Math.max(
                Math.toRadians(RobotConstants.Turret.ANGLE_EXIT_TOLERANCE_DEGREES),
                enterTolerance);
        onTarget = absError <= enterTolerance
                || (onTarget && !targetMoving && absError <= exitTolerance);

        double power = calculateHybridPower(angleErrorRad, targetAngularVelocityRadSec,
                feedforwardDt);

        // Hard cutoff if already at/beyond a soft limit and still pushing outward
        if (useSoftLimits) {
            if ((currentAngleRad >= maxAngleRadians && power > 0.0)
                    || (currentAngleRad <= minAngleRadians && power < 0.0)) {
                power = 0.0;
            }
        }

        double maxPower = RobotConstants.Turret.MAX_POWER;
        setServoPower(Range.clip(power, -maxPower, maxPower));
    }

    /**
     * The hybrid controller: one servo power from the angle error and the target's motion.
     * Uses {@link #onTarget}, {@link #targetMovingLatched} and the unfiltered target speed,
     * which {@link #driveToRelativeAngle} updates first. The caller applies the soft limits
     * and the MAX_POWER cap to the result.
     *
     * @param errorRad             signed angle error, radians (target minus measured,
     *                             already wrapped to the way the turret should go)
     * @param targetVelocityRadSec filtered target angular velocity, radians/second
     * @param dt                   seconds since the previous call, or 0 if unusable
     * @return servo power, + = counterclockwise
     */
    private double calculateHybridPower(double errorRad, double targetVelocityRadSec, double dt) {
        double feedforward = updateFeedforward(targetVelocityRadSec, dt);

        if (onTarget) {
            // Hold. Stationary target: no power at all. A target that is still sweeping
            // (chassis turning or driving past the hive): only keep pace with it, otherwise
            // the turret stops, falls behind, jumps and stops again.
            if (!RobotConstants.Turret.USE_FEEDFORWARD || !targetMovingLatched) {
                return 0.0;
            }
            return feedforward + Math.signum(targetVelocityRadSec) * kF;
        }

        double errorSign = Math.signum(errorRad);
        double power = errorSign * correctionPower(Math.abs(errorRad)) + feedforward;

        // The correction alone is never weaker than the minimum power, so a weak push toward
        // the target means feedforward is opposing it: the turret is AHEAD of a target that
        // is travelling toward it. Wait for the target instead of creeping or kicking (that
        // chatters). If the target has really stopped and only the filtered speed lags
        // behind, push with the minimum power so the command can still move the turret.
        double minPower = minimumPower();
        if (Math.abs(power) < minPower && Math.signum(power) == errorSign) {
            power = isTargetApproaching(errorSign) ? 0.0 : errorSign * minPower;
        }
        return power;
    }

    /**
     * Power that corrects an angle error, from its size alone. Continuous: no step between
     * the zones. See the diagram at {@link RobotConstants.Turret#FAR_ERROR_DEGREES}.
     * Values set out of order in Panels are clamped so that
     * minimum &lt;= fine &lt;= far &lt;= MAX_POWER.
     *
     * @param absErrorRad size of the angle error, radians
     * @return unsigned servo power
     */
    private static double correctionPower(double absErrorRad) {
        double maxPower = RobotConstants.Turret.MAX_POWER;
        double minPower = minimumPower();
        double finePower = Range.clip(RobotConstants.Turret.FINE_POWER, minPower, maxPower);
        double farPower = Range.clip(RobotConstants.Turret.FAR_POWER, finePower, maxPower);

        double tolerance = Math.max(0.0, RobotConstants.Turret.ANGLE_TOLERANCE_DEGREES);
        double fineStart = Math.max(RobotConstants.Turret.FINE_ERROR_DEGREES, tolerance);
        double farStart = Math.max(RobotConstants.Turret.FAR_ERROR_DEGREES, fineStart);
        double error = Math.toDegrees(absErrorRad);

        if (error >= farStart) {
            return farPower;
        }
        if (error > fineStart) {
            double exponent = RobotConstants.Turret.SLOWDOWN_CURVE_EXPONENT;
            if (!(exponent > 0.0)) {
                exponent = 1.0;
            }
            double fraction = (error - fineStart) / (farStart - fineStart);
            return finePower + (farPower - finePower) * Math.pow(fraction, exponent);
        }
        if (error > tolerance) {
            double fraction = (error - tolerance) / (fineStart - tolerance);
            return minPower + (finePower - minPower) * fraction;
        }
        // Inside the enter tolerance but not aimed cannot happen in the control loop.
        return minPower;
    }

    /** @return the smallest correcting power, never above MAX_POWER (low during first tests) */
    private static double minimumPower() {
        return Math.min(RobotConstants.Turret.MIN_COMMAND_POWER, RobotConstants.Turret.MAX_POWER);
    }

    /**
     * Velocity feedforward with a rate limit, so a noisy speed estimate cannot jerk the
     * turret. It may FALL at once (feedforward that lingers overshoots), but it builds up no
     * faster than {@link RobotConstants.Turret#FEEDFORWARD_SLEW_PER_SECOND}, and after a
     * reversal it builds up again from zero. None while the aim is held at a soft limit.
     */
    private double updateFeedforward(double targetVelocityRadSec, double dt) {
        double wanted = 0.0;
        if (RobotConstants.Turret.USE_FEEDFORWARD && targetMovingLatched && !targetClipped) {
            wanted = kV * targetVelocityRadSec;
        }
        double slew = RobotConstants.Turret.FEEDFORWARD_SLEW_PER_SECOND;
        if (slew > 0.0) {
            double kept = (wanted * feedforwardPower > 0.0) ? feedforwardPower : 0.0;
            if (Math.abs(wanted) > Math.abs(kept)) {
                double step = slew * dt;
                wanted = kept + Range.clip(wanted - kept, -step, step);
            }
        }
        feedforwardPower = wanted;
        return feedforwardPower;
    }

    /**
     * @param errorSign direction of the angle error
     * @return true if the target is really (filtered AND unfiltered speed) travelling
     *         against the error, that is, toward the turret
     */
    private boolean isTargetApproaching(double errorSign) {
        double velocitySign = Math.signum(targetAngularVelocityRadSec);
        double movingEnter = Math.toRadians(RobotConstants.Turret.TARGET_MOVING_THRESHOLD_DEG_SEC);
        return targetMovingLatched
                && velocitySign != errorSign
                && Math.signum(rawTargetVelocityRadSec) == velocitySign
                && Math.abs(rawTargetVelocityRadSec) > 0.5 * movingEnter;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /**
     * Sends the same power to both servos. {@code power} is in turret terms (+ =
     * counterclockwise); the runtime {@link #servoSign} correction is applied here.
     */
    private void setServoPower(double power) {
        if (!isFinite(power)) {
            power = 0.0; // a bad Panels value must never reach the servos
        }
        lastCommandedPower = power;
        leftServo.setPower(power * servoSign);
        rightServo.setPower(power * servoSign);
    }

    /**
     * Safety net for the feedback loop. Runs every {@link #update(Pose)}, before the
     * servos are commanded, and compares the measured angle with the power commanded on
     * the previous loop. See {@link #monitorStall} and {@link #monitorDirection}.
     */
    private void monitorTurret() {
        double dt = monitorLoopTimer.seconds();
        monitorLoopTimer.reset();
        double velocityRadSec = (dt > 1e-4 && dt <= 0.5)
                ? (currentAngleRad - previousMonitorAngleRad) / dt : 0.0;
        previousMonitorAngleRad = currentAngleRad;

        boolean closedLoop = mode == TurretMode.HOME || mode == TurretMode.TRACKING;
        monitorStall(lastCommandedPower, closedLoop);
        monitorDirection(lastCommandedPower, velocityRadSec, closedLoop);
    }

    /**
     * Encoder not counting. While HOME or TRACKING commands any power meant to move the
     * turret ({@link RobotConstants.Turret#MIN_COMMAND_POWER} or more) in one direction,
     * the count must keep changing. If it moves by less than
     * {@link RobotConstants.Turret#ENCODER_STALL_MIN_TICKS} for
     * {@link RobotConstants.Turret#ENCODER_STALL_SECONDS} (strong command) or
     * {@link RobotConstants.Turret#ENCODER_STALL_SLOW_SECONDS} (weak command), the turret
     * is not being measured or cannot move: latch {@link #encoderFault}.
     *
     * <p>This has to cover the WEAK commands too. With a count that never changes and the
     * robot roughly facing the hive, the error is a constant degree or two and the command
     * is the constant minimum power: the turret turns slow circles forever. Only a check
     * that works at the minimum power can catch that.
     *
     * <p>MANUAL is not checked: the jog is open-loop and must keep working without the
     * encoder.
     */
    private void monitorStall(double power, boolean closedLoop) {
        double minPower = Math.min(RobotConstants.Turret.MIN_COMMAND_POWER,
                RobotConstants.Turret.MAX_POWER) - 1e-6;
        double sign = (closedLoop && Math.abs(power) >= minPower) ? Math.signum(power) : 0.0;
        boolean progress = Math.abs(lastEncoderTicks - stallStartTicks)
                >= RobotConstants.Turret.ENCODER_STALL_MIN_TICKS;
        if (sign == 0.0 || sign != stallPowerSign || progress) {
            // Not driving, direction changed, or the count moved: start a fresh window.
            stallPowerSign = sign;
            stallStartTicks = lastEncoderTicks;
            stallTimer.reset();
            return;
        }
        double limitSeconds = Math.abs(power) >= RobotConstants.Turret.ENCODER_STALL_MIN_POWER
                ? RobotConstants.Turret.ENCODER_STALL_SECONDS
                : RobotConstants.Turret.ENCODER_STALL_SLOW_SECONDS;
        if (stallTimer.seconds() >= limitSeconds) {
            encoderFault = true;
            directionFault = false;
            restartMonitor();
        }
    }

    /**
     * Servo and encoder directions disagree. A command at or above
     * {@link RobotConstants.Turret#DIRECTION_MONITOR_MIN_POWER} must move the measured
     * angle THAT way. If the angle instead travels the other way by
     * {@link RobotConstants.Turret#RUNAWAY_DEGREES} plus the coast allowance, the loop is
     * running backwards (every correction pushes the turret further from the target).
     *
     * <ul>
     *   <li>HOME / TRACKING: flip the runtime servo sign so the loop is stable again and
     *       report it. After {@link RobotConstants.Turret#MAX_AUTO_FLIPS} flips, stop the
     *       turret instead: flipping is not fixing it.</li>
     *   <li>MANUAL: only report it. Reversing the servos in the middle of a jog would turn
     *       the turret the other way under the operator's thumb.</li>
     * </ul>
     *
     * <p>Coast allowance: when the command reverses while the turret is still moving (it
     * overshot the target after a fast move), the turret keeps going the "wrong" way for
     * a moment. That is not a reversed servo. The allowed wrong-way travel is therefore
     * raised by the turret's speed at that moment times
     * {@link RobotConstants.Turret#DIRECTION_MONITOR_COAST_SECONDS}.
     */
    private void monitorDirection(double power, double velocityRadSec, boolean closedLoop) {
        double runawayRad = Math.toRadians(RobotConstants.Turret.RUNAWAY_DEGREES);
        double sign = Math.abs(power) >= RobotConstants.Turret.DIRECTION_MONITOR_MIN_POWER
                ? Math.signum(power) : 0.0;
        if (sign == 0.0 || sign != directionPowerSign) {
            // New command direction (or no real command): start a window from here.
            boolean reversed = sign != 0.0 && directionPowerSign == -sign;
            if (closedLoop && reversed && Math.abs(angleErrorRad) > Math.PI / 2.0
                    && commandReversedFarFromTarget()) {
                directionPowerSign = 0.0;
                directionsDisagree();
                return;
            }
            directionPowerSign = sign;
            directionStartAngleRad = currentAngleRad;
            double speedAgainstCommand = Math.max(0.0, -sign * velocityRadSec);
            directionAllowedRad = runawayRad
                    + speedAgainstCommand * RobotConstants.Turret.DIRECTION_MONITOR_COAST_SECONDS;
            return;
        }

        double movedRad = sign * (currentAngleRad - directionStartAngleRad); // + = the commanded way
        if (movedRad >= runawayRad) {
            // Moving the right way. Start a fresh window from here, with no coast allowance.
            directionStartAngleRad = currentAngleRad;
            directionAllowedRad = runawayRad;
        } else if (movedRad <= -directionAllowedRad) {
            // Pushed one way, went the other way.
            directionPowerSign = 0.0; // the next loop opens a new window at the current speed
            if (closedLoop) {
                directionsDisagree();
            } else {
                jogDirectionMismatch = true;
            }
        }
    }

    /**
     * Counts command reversals that happen more than 90 degrees from the target (see
     * {@link RobotConstants.Turret#ANTIPODE_CHATTER_REVERSALS}).
     *
     * @return true once there were enough of them, close enough together, to mean the
     *         turret is buzzing at the point opposite the target
     */
    private boolean commandReversedFarFromTarget() {
        if (farReversalCount == 0
                || farReversalTimer.seconds() > RobotConstants.Turret.ANTIPODE_CHATTER_SECONDS) {
            farReversalCount = 0;
            farReversalTimer.reset();
        }
        farReversalCount++;
        if (farReversalCount >= RobotConstants.Turret.ANTIPODE_CHATTER_REVERSALS) {
            farReversalCount = 0;
            return true;
        }
        return false;
    }

    /**
     * HOME / TRACKING found the servo and encoder directions disagreeing: flip the
     * runtime servo sign, or stop the turret if that was already tried
     * {@link RobotConstants.Turret#MAX_AUTO_FLIPS} times.
     */
    private void directionsDisagree() {
        if (autoFlipCount >= RobotConstants.Turret.MAX_AUTO_FLIPS) {
            encoderFault = true;
            directionFault = true;
        } else {
            servoSign = -servoSign;
            autoFlipCount++;
            feedforwardPower = 0.0;
        }
    }

    /** Forgets both monitor windows. Called when the zero, the mode or the fault changes. */
    private void restartMonitor() {
        directionPowerSign = 0.0;
        directionStartAngleRad = currentAngleRad;
        directionAllowedRad = Math.toRadians(RobotConstants.Turret.RUNAWAY_DEGREES);
        farReversalCount = 0;
        stallPowerSign = 0.0;
        stallStartTicks = lastEncoderTicks;
        stallTimer.reset();
        previousMonitorAngleRad = currentAngleRad;
        monitorLoopTimer.reset();
    }

    /**
     * Operator jog. Switches to {@link TurretMode#MANUAL} and drives the servos with a
     * fixed power. Call it every loop a jog button is held, and call {@code jog(0.0)}
     * when it is released: the turret then stops and stays where it is, still in MANUAL,
     * until another mode is selected. Open-loop, so it works without the encoder.
     *
     * @param power servo power, + = counterclockwise (left from above), 0 = stop
     */
    public void jog(double power) {
        if (mode != TurretMode.MANUAL) {
            setMode(TurretMode.MANUAL);
        }
        manualPower = power;
    }

    /**
     * @return true if HOME and TRACKING are stopped because the monitor saw the encoder
     *         not counting, or the direction flipping over and over (see
     *         {@link #clearFault()}). MANUAL jog still works.
     */
    public boolean hasEncoderFault() {
        return encoderFault;
    }

    /** Clears {@link #hasEncoderFault()} so HOME and TRACKING drive the turret again. */
    public void clearFault() {
        encoderFault = false;
        directionFault = false;
        restartMonitor();
    }

    /** @return how many times the servo direction was flipped at run time (0 = flags are right) */
    public int getAutoFlipCount() {
        return autoFlipCount;
    }

    /** @return true if the servos are currently driven opposite to the RobotConstants flags */
    public boolean isServoDirectionAutoFlipped() {
        return servoSign < 0;
    }

    /**
     * Manually sets the target coordinate (Pedro inches). This turns OFF automatic hive-side
     * selection; call {@link #setAutoSelectSide(boolean)} with true to turn it back on.
     */
    public void setTargetCoordinate(double x, double y) {
        if (autoSelectSide || x != targetXInches || y != targetYInches) {
            // The target is about to jump: do not read the jump as target velocity,
            // and the turret is not aimed at the NEW target until update() says so.
            firstTargetSample = true;
            onTarget = false;
        }
        this.autoSelectSide = false;
        this.targetXInches = x;
        this.targetYInches = y;
    }

    /**
     * Picks the upper or lower hive side from the robot's Y position.
     *
     * <p>The first time (and after automatic selection is switched back on) the side is
     * chosen by which half of the field the robot is in. After that a small hysteresis
     * band around the midline stops the target from flipping back and forth when the
     * robot is driving along the line.
     */
    private void selectTargetSide(Pose pose) {
        if (!autoSelectSide || pose == null) {
            return;
        }
        double y = pose.y();
        double mid = RobotConstants.Turret.SIDE_SWITCH_Y_INCHES;
        double band = RobotConstants.Turret.SIDE_HYSTERESIS_INCHES;

        boolean top = aimingAtTopSide;
        if (!sideChosen) {
            top = y >= mid;
        } else if (y > mid + band) {
            top = true;
        } else if (y < mid - band) {
            top = false;
        }

        if (sideChosen && top != aimingAtTopSide) {
            // Side switch: the target jumps. Do not read the jump as target velocity.
            firstTargetSample = true;
        }
        aimingAtTopSide = top;
        sideChosen = true;

        targetXInches = aimingAtTopSide
                ? RobotConstants.Turret.TOP_TARGET_X_INCHES : RobotConstants.Turret.BOTTOM_TARGET_X_INCHES;
        targetYInches = aimingAtTopSide
                ? RobotConstants.Turret.TOP_TARGET_Y_INCHES : RobotConstants.Turret.BOTTOM_TARGET_Y_INCHES;
    }

    /** Turns automatic upper/lower hive side selection on or off. */
    public void setAutoSelectSide(boolean autoSelectSide) {
        if (autoSelectSide && !this.autoSelectSide) {
            // Coming back from a fixed target: pick the side fresh from the robot's
            // position, and do not read the jump back to the hive as target velocity.
            sideChosen = false;
            firstTargetSample = true;
            onTarget = false;
        }
        this.autoSelectSide = autoSelectSide;
    }

    /** @return true if hive side selection is automatic (false while a fixed target is set) */
    public boolean isAutoSelectSide() {
        return autoSelectSide;
    }

    /** @return true if the turret is currently aiming at the upper hive side */
    public boolean isAimingAtTopSide() {
        return aimingAtTopSide;
    }

    public double getTargetX() {
        return targetXInches;
    }

    public double getTargetY() {
        return targetYInches;
    }

    public void setMode(TurretMode mode) {
        if (this.mode != mode) {
            feedforwardPower = 0.0;
            firstTargetSample = true;
            onTarget = false;
            restartMonitor();
            if (mode == TurretMode.TRACKING) {
                // Entering TRACKING: pick the hive side fresh from where the robot is now.
                // The side is not updated while HOME or MANUAL is active, so the old pick
                // (and its hysteresis) may belong to the other half of the field.
                sideChosen = false;
            }
        }
        if (mode != TurretMode.MANUAL) {
            manualPower = 0.0;
        }
        this.mode = mode;
        if (mode == TurretMode.DISABLED) {
            stop();
        }
    }

    /**
     * Sets the chassis-relative angle the turret holds in {@link TurretMode#HOME}
     * (0 = straight forward, positive = counterclockwise / left).
     *
     * @param angleRadians angle to hold, in radians
     */
    public void setHomeAngle(double angleRadians) {
        if (angleRadians != homeAngleRad) {
            // The held angle is about to jump: do not read the jump as target velocity.
            firstTargetSample = true;
            onTarget = false;
        }
        this.homeAngleRad = angleRadians;
    }

    public TurretMode getMode() {
        return mode;
    }

    /** @return the chassis-relative angle (radians) held in {@link TurretMode#HOME} */
    public double getHomeAngleRadians() {
        return homeAngleRad;
    }

    /**
     * Distance in inches from the turret pivot to the current target. Pure getter: it
     * does not change which hive side is selected.
     */
    public double getDistanceToTarget(Pose currentPose) {
        if (currentPose == null) {
            return 0.0;
        }
        double heading = currentPose.heading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double turretWorldX = currentPose.x() + (xOffsetInches * cosH - yOffsetInches * sinH);
        double turretWorldY = currentPose.y() + (xOffsetInches * sinH + yOffsetInches * cosH);

        return Math.hypot(targetXInches - turretWorldX, targetYInches - turretWorldY);
    }

    /**
     * Changes the feedforward gains at run time (the power profile is in RobotConstants).
     *
     * @param kI ignored: the hybrid controller has no integral term
     * @param kD ignored: the hybrid controller has no derivative term
     * @param kV velocity feedforward, power per rad/s of target motion
     * @param kF breakaway power used to keep pace with a moving target while aimed
     */
    public void setGains(double kI, double kD, double kV, double kF) {
        this.kV = kV;
        this.kF = kF;
    }

    public void setSoftLimitsDegrees(boolean useSoftLimits, double minDegrees, double maxDegrees) {
        this.useSoftLimits = useSoftLimits;
        this.minAngleRadians = Math.toRadians(minDegrees);
        this.maxAngleRadians = Math.toRadians(maxDegrees);
    }

    public void setMountingOffset(double xOffsetInches, double yOffsetInches) {
        this.xOffsetInches = xOffsetInches;
        this.yOffsetInches = yOffsetInches;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            stop();
        }
    }

    /** Stops both servos and resets the controller. The turret is no longer "on target". */
    public void stop() {
        feedforwardPower = 0.0;
        firstTargetSample = true;
        onTarget = false;
        targetMovingLatched = false;
        setServoPower(0.0);
    }

    /**
     * @return measured chassis-relative turret angle (radians, continuous, + = counterclockwise)
     *         as of the last {@link #update(Pose)} or {@link #refresh()}
     */
    public double getCurrentAngleRadians() {
        return currentAngleRad;
    }

    public double getCurrentAngleDegrees() {
        return Math.toDegrees(currentAngleRad);
    }

    /** @return raw encoder ticks from the last read, before the zero offset is applied */
    public int getEncoderTicks() {
        return lastEncoderTicks;
    }

    public double getTargetAngleRadians() {
        return relativeTargetAngleRad;
    }

    public double getTargetAngleDegrees() {
        return Math.toDegrees(relativeTargetAngleRad);
    }

    public double getAngleErrorRadians() {
        return angleErrorRad;
    }

    public double getAngleErrorDegrees() {
        return Math.toDegrees(angleErrorRad);
    }

    public double getLastCommandedPower() {
        return lastCommandedPower;
    }

    /**
     * @return field angle (degrees) from the turret pivot to the target, as of the last
     *         {@link #update(Pose)} in TRACKING mode. This is the number that must change
     *         when the robot translates.
     */
    public double getTargetGlobalDegrees() {
        return Math.toDegrees(targetGlobalAngleRad);
    }

    /**
     * @return one-line turret state for the Driver Station: the mode and what it is aiming
     *         at, or what is keeping the servos stopped
     */
    public String getStatusText() {
        if (!enabled || mode == TurretMode.DISABLED) {
            return "DISABLED (stopped)";
        }
        if (mode == TurretMode.MANUAL) {
            return "MANUAL";
        }
        if (encoderFault) {
            return "STOPPED BY FAULT (was " + mode + ")";
        }
        if (mode == TurretMode.HOME) {
            return String.format("HOME (fixed %.0f deg, not the hive)", Math.toDegrees(homeAngleRad));
        }
        if (autoSelectSide) {
            return aimingAtTopSide ? "TRACKING HIVE TOP" : "TRACKING HIVE BOTTOM";
        }
        return "TRACKING FIXED POINT";
    }

    /**
     * Adds the turret state to the Driver Station, plus one short line for each thing that
     * is wrong. The caller still calls {@code telemetry.update()}.
     *
     * <p>How to fix each warning (after the match):
     * <ul>
     *   <li>{@code DIRECTION KEEPS FLIPPING}: the measured angle does not follow the servos.
     *       Check that the encoder gear is tight and that motor port
     *       {@link RobotConstants.Turret#ENCODER_PORT} really carries the turret encoder.</li>
     *   <li>{@code ENCODER NOT COUNTING}: the servos were driven and the count did not
     *       change. Either the encoder cable is not in that port's encoder socket (fix
     *       {@code ENCODER_PORT}; the init screen shows which port counts), or the turret
     *       cannot turn (jammed, or raise
     *       {@link RobotConstants.Turret#MIN_COMMAND_POWER}).</li>
     *   <li>{@code SERVO DIRECTION AUTO-FLIPPED} or {@code JOG DIRECTION WRONG}: the servo
     *       and encoder directions disagree. On the init screen turn the turret LEFT by
     *       hand. Angle goes UP: the encoder is right, so flip BOTH {@code SERVO_*_REVERSED}.
     *       Angle goes DOWN: flip {@code ENCODER_REVERSED} and leave the servo flags.</li>
     * </ul>
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret", getStatusText());
        if (encoderFault) {
            telemetry.addLine(directionFault
                    ? "!! TURRET STOPPED: DIRECTION KEEPS FLIPPING (G2 Dpad Down or Up retries)"
                    : "!! TURRET STOPPED: ENCODER NOT COUNTING (G2 Dpad Down or Up retries)");
        }
        if (autoFlipCount > 0) {
            telemetry.addLine("!! SERVO DIRECTION AUTO-FLIPPED");
        }
        if (jogDirectionMismatch) {
            telemetry.addLine("!! JOG DIRECTION WRONG");
        }
    }

    /**
     * @return true if the turret counts as aimed at the angle it was told to hold. It has
     *         hysteresis: it turns true once the error is inside
     *         {@link RobotConstants.Turret#ANGLE_TOLERANCE_DEGREES} and, while the target is
     *         not moving, stays true until the error exceeds
     *         {@link RobotConstants.Turret#ANGLE_EXIT_TOLERANCE_DEGREES}.
     *         In HOME mode that angle is the home angle, NOT the hive; use
     *         {@link #isAimedAtTarget()} to decide whether to shoot. False before the first
     *         update, whenever the turret is stopped, and from a target change until the
     *         next update.
     */
    public boolean isOnTarget() {
        return onTarget;
    }

    /**
     * @return true only if the turret is TRACKING the field target, {@link #isOnTarget()}
     *         is true (so, once the turret has settled on a stationary target, it is within
     *         the exit tolerance), and the aim angle did not have to be clipped to a soft
     *         limit. This is the "safe to shoot" check.
     */
    public boolean isAimedAtTarget() {
        return enabled && !encoderFault && mode == TurretMode.TRACKING && onTarget && !targetClipped;
    }
}