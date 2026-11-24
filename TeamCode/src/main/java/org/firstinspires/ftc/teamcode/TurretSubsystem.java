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
 * working without the encoder.
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
 */
public class TurretSubsystem {

    /**
     * Turret power controller. The main term is NOT a plain proportional gain: it is the
     * turning speed profile from {@link TurretSubsystem#approachPower(double)} (full power
     * far away, then a tunable slow-down). On top of that this class adds the optional
     * integral and derivative terms and the velocity feedforward (kV), and it holds the
     * breakaway power (kF) used when only keeping pace with a moving target.
     */
    public static class PIDFController {
        private double kI;
        private double kD;
        private double kF;
        private double maxIntegral = RobotConstants.Turret.MAX_INTEGRAL;

        private double integralSum = 0.0;
        private double prevError = 0.0;
        private boolean firstUpdate = true;
        private final ElapsedTime timer = new ElapsedTime();

        public PIDFController(double kI, double kD, double kF) {
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        /**
         * @param angleError           signed angle error, radians (target minus measured)
         * @param approachPower        signed power from the turning speed profile for this error
         * @param targetVelocityRadSec target angular velocity, radians/second
         * @param kV                   velocity feedforward gain (power per rad/s)
         * @return servo power, not yet clipped
         */
        public double calculate(double angleError, double approachPower,
                                double targetVelocityRadSec, double kV) {
            double dt = timer.seconds();
            timer.reset();

            if (firstUpdate || dt <= 0.0 || dt > 0.5) {
                dt = 0.0;
                firstUpdate = false;
            }

            if (dt > 0.0 && kI != 0.0) {
                integralSum = Range.clip(integralSum + angleError * dt, -maxIntegral, maxIntegral);
            }
            double iTerm = kI * integralSum;

            double derivative = (dt > 0.0) ? (angleError - prevError) / dt : 0.0;
            double dTerm = kD * derivative;
            prevError = angleError;

            double vTerm = kV * targetVelocityRadSec;

            return approachPower + iTerm + dTerm + vTerm;
        }

        /**
         * Power needed just to keep pace with a target moving at the given speed, with no
         * correction for angle error: velocity feedforward plus the breakaway power in the
         * direction of travel. Used while the turret is already inside tolerance.
         *
         * @param targetVelocityRadSec target angular velocity, radians/second
         * @param kV                   velocity feedforward gain (power per rad/s)
         * @return servo power, not yet clipped
         */
        public double feedforward(double targetVelocityRadSec, double kV) {
            return kV * targetVelocityRadSec + Math.signum(targetVelocityRadSec) * kF;
        }

        public void setGains(double kI, double kD, double kF) {
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public void reset() {
            integralSum = 0.0;
            prevError = 0.0;
            firstUpdate = true;
            timer.reset();
        }

        /**
         * Restarts the loop timing but KEEPS the integral. Call while the PID output is
         * not being used (turret aimed) so the next {@link #calculate} does not see a
         * stale time step.
         */
        public void resetTiming() {
            firstUpdate = true;
            timer.reset();
        }
    }

    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx turretEncoder;
    private final PIDFController pidfController;
    private final ElapsedTime targetVelTimer = new ElapsedTime();
    private final ElapsedTime robotMotionTimer = new ElapsedTime();
    private final ElapsedTime robotStopTimer = new ElapsedTime();

    /** High-level turret operating mode. */
    public enum TurretMode {
        /** Continuously aims at the field target coordinate. */
        TRACKING,
        /**
         * Turns to and holds a fixed chassis-relative angle
         * ({@link RobotConstants.Turret#HOME_ANGLE_RADIANS} unless changed with
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
    private double homeAngleRad = RobotConstants.Turret.HOME_ANGLE_RADIANS;

    // Geometry
    private double xOffsetInches = RobotConstants.Turret.OFFSET_X_INCHES;
    private double yOffsetInches = RobotConstants.Turret.OFFSET_Y_INCHES;

    private double previousRobotX = 0.0;
    private double previousRobotY = 0.0;
    private double previousRobotHeading = 0.0;
    private boolean hasPreviousRobotPose = false;
    private boolean robotMotionInitialized = false;
    private boolean robotMoving = false;
    private boolean positionHoldActive = false;
    private double lockedAngleRad = 0.0;

    // Encoder zero. Stored as an offset instead of resetting the hub encoder, because the
    // encoder shares its port with a real motor and because this makes resetAngle() cheap
    // and safe to call at any time.
    private int zeroOffsetTicks = 0;
    private int lastEncoderTicks = 0;

    // Field target (hive) in Pedro Pathing inches
    private double targetXInches = RobotConstants.Turret.DEFAULT_TARGET_X_INCHES;
    private double targetYInches = RobotConstants.Turret.DEFAULT_TARGET_Y_INCHES;

    // The hive has two sides. By default the side is picked from the robot's Y position.
    private boolean autoSelectSide = true;
    private boolean aimingAtTopSide = false;
    /** False until the side has been picked from a real pose (then hysteresis applies). */
    private boolean sideChosen = false;

    private double kV = RobotConstants.Turret.KV;

    // Internal tracking state
    private double previousTargetAngleRad = 0.0;
    private double targetAngularVelocityRadSec = 0.0; // low-pass filtered
    private double rawTargetVelocityRadSec = 0.0;     // unfiltered, this loop only
    private boolean firstTargetSample = true;
    private boolean enabled = true;

    private double currentAngleRad = 0.0;
    private double targetGlobalAngleRad = 0.0;
    private double relativeTargetAngleRad = 0.0;
    private double angleErrorRad = 0.0;
    private double lastCommandedPower = 0.0;

    /** True only while the last update() measured the turret inside tolerance. */
    private boolean onTarget = false;
    /** True if the last commanded angle had to be clipped to a soft limit. */
    private boolean targetClipped = false;

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

        pidfController = new PIDFController(
                RobotConstants.Turret.KI,
                RobotConstants.Turret.KD,
                RobotConstants.Turret.KF
        );
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
        currentAngleRad = readAngleRadians();
    }

    /**
     * Reads the encoder and converts ticks to a chassis-relative turret angle.
     * The result is continuous (it keeps counting past +/- PI); closed-loop commands
     * use the shortest wrapped error to reach the target.
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
     * Call once per loop. Computes the chassis-relative angle to the target from the
     * Pedro pose and drives the servos with PIDF control on the encoder angle.
     *
     * @param currentPose current robot {@link Pose} from Pedro Pathing
     */
    public void update(Pose currentPose) {
        updateInternal(currentPose, null);
    }

    /**
     * Call once per loop, optionally supplying drivetrain motion state instead of
     * estimating it from pose deltas.
     *
     * @param currentPose current robot {@link Pose} from Pedro Pathing
     * @param isRobotMoving true while the chassis is moving
     */
    public void update(Pose currentPose, boolean isRobotMoving) {
        updateInternal(currentPose, isRobotMoving);
    }

    private void updateInternal(Pose currentPose, Boolean explicitRobotMoving) {
        currentAngleRad = readAngleRadians();
        monitorTurret();
        updateRobotMotion(currentPose, explicitRobotMoving);

        if (!enabled || mode == TurretMode.DISABLED) {
            stop();
            return;
        }

        // MANUAL comes before the fault check on purpose: the jog is open-loop, so it is
        // the operator's way to move the turret when the encoder is not working.
        if (mode == TurretMode.MANUAL) {
            onTarget = false;
            setServoPower(Range.clip(manualPower, -RobotConstants.Turret.MAX_POWER,
                    RobotConstants.Turret.MAX_POWER));
            return;
        }

        // HOME and TRACKING aim with the measured angle. Without a working measurement
        // they would only spin the turret, so they stay stopped until the fault is cleared.
        if (encoderFault) {
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

        if (positionHoldActive) {
            driveToRelativeAngle(lockedAngleRad);
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

    private void updateRobotMotion(Pose currentPose, Boolean explicitRobotMoving) {
        double dt = robotMotionTimer.seconds();
        robotMotionTimer.reset();

        boolean estimatedMoving = false;
        if (currentPose != null) {
            if (hasPreviousRobotPose && dt > 1e-4) {
                double linearSpeed = Math.hypot(
                        currentPose.x() - previousRobotX,
                        currentPose.y() - previousRobotY) / dt;
                double angularSpeed = Math.abs(AngleUnit.normalizeRadians(
                        currentPose.heading() - previousRobotHeading)) / dt;
                estimatedMoving = linearSpeed
                        > RobotConstants.Turret.ROBOT_LINEAR_SPEED_THRESHOLD_IN_PER_SEC
                        || angularSpeed
                        > RobotConstants.Turret.ROBOT_ANGULAR_SPEED_THRESHOLD_RAD_PER_SEC;
            }
            previousRobotX = currentPose.x();
            previousRobotY = currentPose.y();
            previousRobotHeading = currentPose.heading();
            hasPreviousRobotPose = true;
        }

        boolean moving = explicitRobotMoving != null ? explicitRobotMoving : estimatedMoving;
        robotMoving = moving;
        if (mode != TurretMode.TRACKING) {
            positionHoldActive = false;
            robotMotionInitialized = false;
            return;
        }
        if (currentPose == null) {
            if (moving && positionHoldActive) {
                firstTargetSample = true;
            }
            positionHoldActive = false;
            robotMotionInitialized = false;
            robotStopTimer.reset();
            return;
        }

        if (moving) {
            robotStopTimer.reset();
            robotMotionInitialized = true;
            if (positionHoldActive) {
                positionHoldActive = false;
                firstTargetSample = true;
            }
        } else {
            if (!robotMotionInitialized) {
                robotStopTimer.reset();
                robotMotionInitialized = true;
            }
            if (!positionHoldActive
                    && robotStopTimer.seconds()
                            >= RobotConstants.Turret.ROBOT_STOP_DEBOUNCE_SECONDS) {
                lockedAngleRad = currentAngleRad;
                positionHoldActive = true;
                firstTargetSample = true;
            }
        }
    }

    private void driveToRelativeAngle(double relativeTargetAngle) {
        relativeTargetAngleRad = relativeTargetAngle;

        // Target angular velocity (rad/s) for kV feedforward. It is capped at the turret's
        // top speed and low-pass filtered, so one jittery loop time cannot spike the
        // feedforward. A deliberate target jump (side switch, new target, new home angle)
        // sets firstTargetSample so the jump is not mistaken for a fast-moving target.
        double dt = targetVelTimer.seconds();
        targetVelTimer.reset();
        double maxVelocity = RobotConstants.Turret.RAD_PER_SEC_AT_FULL_POWER;
        if (!firstTargetSample && dt > 1e-4 && dt <= 0.5) {
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
        boolean targetMoving = Math.abs(targetAngularVelocityRadSec)
                > RobotConstants.Turret.TARGET_MOVING_THRESHOLD_RAD_SEC;

        // Shortest path from the measured angle to the target, across +/- PI and after
        // any number of full revolutions.
        angleErrorRad = AngleUnit.normalizeRadians(relativeTargetAngleRad - currentAngleRad);

        onTarget = Math.abs(angleErrorRad) <= RobotConstants.Turret.ANGLE_TOLERANCE_RADIANS;

        double power;
        if (onTarget) {
            // Aimed. The PID is idle here: restart its timing but keep the integral, so a
            // slowly moving target does not restart the controller from zero every time
            // the turret catches up.
            pidfController.resetTiming();
            if (targetMoving) {
                // The target is still sweeping (chassis turning or driving past the hive).
                // Keep pace with feedforward instead of stopping dead, otherwise the
                // turret stops, falls behind, jumps and stops again.
                power = pidfController.feedforward(targetAngularVelocityRadSec, kV);
            } else {
                power = 0.0;
            }
        } else {
            // Main term: the turning speed profile (flat out far away, then slow down).
            power = pidfController.calculate(angleErrorRad, approachPower(angleErrorRad),
                    targetAngularVelocityRadSec, kV);

            // Never leave the turret outside tolerance with a correction too weak to move
            // it. Only a WEAK command (below the minimum power) is touched here.
            double minPower = RobotConstants.Turret.MIN_COMMAND_POWER;
            if (Math.abs(power) < minPower) {
                double errorSign = Math.signum(angleErrorRad);
                if (!targetMoving) {
                    // Target standing still: always push toward it with the minimum power.
                    power = errorSign * minPower;
                } else if (Math.signum(power) == errorSign) {
                    // Target moving and the controller wants a weak push toward it.
                    double velocitySign = Math.signum(targetAngularVelocityRadSec);
                    boolean targetComingToTurret = velocitySign != errorSign
                            && Math.signum(rawTargetVelocityRadSec) == velocitySign
                            && Math.abs(rawTargetVelocityRadSec)
                                    > 0.5 * RobotConstants.Turret.TARGET_MOVING_THRESHOLD_RAD_SEC;
                    // Turret AHEAD of a target that is still travelling toward it: wait for
                    // the target instead of kicking the turret backwards (that makes it
                    // chatter). Turret BEHIND the target: push with the minimum power.
                    power = targetComingToTurret ? 0.0 : errorSign * minPower;
                }
                // else: moving target and the controller asks for the other direction
                // (velocity feedforward). Leave that command alone.
            }
        }

        double maxPower = RobotConstants.Turret.MAX_POWER;
        setServoPower(Range.clip(power, -maxPower, maxPower));
    }

    /**
     * Turning speed profile: the servo power for a given distance from the target.
     *
     * <ul>
     *   <li>Farther than {@link RobotConstants.Turret#SLOWDOWN_START_DEGREES}: full
     *       {@link RobotConstants.Turret#MAX_POWER}, so big moves are as fast as possible.</li>
     *   <li>Closer than that: the power drops by
     *       {@link RobotConstants.Turret#SLOWDOWN_POWER_PER_DEGREE} for every degree closer
     *       to the target, so the turret brakes before it arrives.</li>
     *   <li>It never drops below {@link RobotConstants.Turret#MIN_COMMAND_POWER}, the
     *       smallest power the code uses to correct toward the target (the servo dead
     *       zone plus a margin).</li>
     *   <li>Safety rule: the slow-down is never gentler than what it takes to reach that
     *       minimum power by the time the turret arrives. A ramp that arrives hotter makes
     *       the turret swing back and forth across the target without stopping, so a
     *       rate that is too small for the chosen zone and MAX_POWER is steepened here.</li>
     * </ul>
     *
     * @param angleErrorRad signed angle error, radians (target minus measured)
     * @return signed servo power pointing toward the target
     */
    private static double approachPower(double angleErrorRad) {
        double maxPower = RobotConstants.Turret.MAX_POWER;
        // Guard against MAX_POWER being set below the minimum during first tests.
        double minPower = Math.min(RobotConstants.Turret.MIN_COMMAND_POWER, maxPower);

        double startDeg = RobotConstants.Turret.SLOWDOWN_START_DEGREES;
        double rate = RobotConstants.Turret.SLOWDOWN_POWER_PER_DEGREE;
        if (startDeg > 0.0) {
            // A ramp too gentle for this zone and MAX_POWER would arrive above the minimum
            // power and swing across the target forever. Steepen it so the power always
            // gets down to the minimum by the time the turret reaches the target.
            rate = Math.max(rate, (maxPower - minPower) / startDeg);
        }

        double distanceDeg = Math.toDegrees(Math.abs(angleErrorRad));
        double degreesIntoSlowZone = startDeg - distanceDeg;

        double power = maxPower;
        if (degreesIntoSlowZone > 0.0) {
            power = maxPower - rate * degreesIntoSlowZone;
        }
        return Math.signum(angleErrorRad) * Range.clip(power, minPower, maxPower);
    }

    /**
     * Sends the same power to both servos. {@code power} is in turret terms (+ =
     * counterclockwise); the runtime {@link #servoSign} correction is applied here.
     */
    private void setServoPower(double power) {
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
            pidfController.reset();
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
            pidfController.reset();
            firstTargetSample = true;
            onTarget = false;
            positionHoldActive = false;
            robotMotionInitialized = false;
            robotStopTimer.reset();
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

    /** Changes the extra controller terms at run time (the slow-down profile is in RobotConstants). */
    public void setGains(double kI, double kD, double kV, double kF) {
        this.kV = kV;
        this.pidfController.setGains(kI, kD, kF);
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

    /** Stops both servos and resets PIDF state. The turret is no longer "on target". */
    public void stop() {
        pidfController.reset();
        firstTargetSample = true;
        onTarget = false;
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
     * @return one-line turret state for the Driver Station: the mode, or what is keeping
     *         the servos stopped
     */
    public String getStatusText() {
        if (!enabled || mode == TurretMode.DISABLED) {
            return "DISABLED (stopped)";
        }
        if (mode == TurretMode.MANUAL) {
            return manualPower == 0.0
                    ? "MANUAL (stopped; Dpad Left/Right moves it)"
                    : String.format("MANUAL (jog power %.2f)", manualPower);
        }
        if (encoderFault) {
            return "STOPPED BY FAULT (was " + mode + ")";
        }
        return mode == TurretMode.HOME ? "HOME (fixed angle on the robot)" : "TRACKING (hive)";
    }

    /**
     * Adds the turret state, warnings and aiming diagnostics to the Driver Station. The
     * caller still calls {@code telemetry.update()}. Put this FIRST on the screen: these
     * are the lines that show why the turret is or is not aiming.
     *
     * <ul>
     *   <li>{@code Turret}: the mode, or what is keeping the servos stopped</li>
     *   <li>{@code encoder}: the motor port the count is read through, and whether the
     *       count has changed at all since Init</li>
     *   <li>{@code aiming at}: HIVE TOP / HIVE BOTTOM, FIXED POINT for a manual target,
     *       FIXED ANGLE in HOME mode, NOTHING when stopped</li>
     *   <li>{@code targetGlobalDeg}: field angle from the turret to the target</li>
     *   <li>{@code relativeTargetDeg}: that angle relative to the chassis (where the turret should point)</li>
     *   <li>{@code measuredAngleDeg}: where the encoder says the turret points</li>
     *   <li>{@code encoderTicks}: raw encoder count (use this to check ENCODER_REVERSED)</li>
     *   <li>{@code angleErrorDeg}: relative target minus measured, shortest path</li>
     *   <li>{@code aimed at target}: true only when TRACKING and inside tolerance</li>
     *   <li>{@code lastCommandedPower}: power sent to both servos this loop</li>
     * </ul>
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret", getStatusText());
        if (encoderFault && directionFault) {
            telemetry.addLine(String.format(
                    "!! TURRET STOPPED: DIRECTION KEEPS FLIPPING (%d flips). The measured angle does"
                            + " not follow the servos. Check that the encoder gear is tight and that"
                            + " motor port '%s' really carries the TURRET encoder. Dpad Left/Right"
                            + " still jogs. G2 Dpad Down or Up retries.",
                    autoFlipCount, RobotConstants.Turret.ENCODER_PORT));
        } else if (encoderFault) {
            telemetry.addLine(String.format(
                    "!! TURRET STOPPED: ENCODER NOT COUNTING. The servos were driven and the count"
                            + " read through motor port '%s' did not change. Either the turret"
                            + " encoder cable is not in THAT port's encoder socket (fix"
                            + " Turret.ENCODER_PORT; the init screen shows which port counts) or"
                            + " the turret cannot turn (jammed, or the minimum power is too weak:"
                            + " Turret.SERVO_DEADBAND_POWER). Dpad Left/Right still jogs. G2 Dpad"
                            + " Down or Up retries.",
                    RobotConstants.Turret.ENCODER_PORT));
        }
        if (autoFlipCount > 0) {
            telemetry.addLine(String.format(
                    "!! SERVO DIRECTION AUTO-FLIPPED (%d x, now %s). Servo and encoder directions"
                            + " disagree. Find the wrong flag on the INIT screen: turn the turret"
                            + " LEFT by hand. Angle goes UP = encoder is right, so set BOTH"
                            + " SERVO_*_REVERSED to %b. Angle goes DOWN = set ENCODER_REVERSED to %b"
                            + " and leave the servo flags.",
                    autoFlipCount, servoSign < 0 ? "reversed" : "as configured",
                    !RobotConstants.Turret.SERVO_LEFT_REVERSED,
                    !RobotConstants.Turret.ENCODER_REVERSED));
        }
        if (jogDirectionMismatch) {
            telemetry.addLine("!! JOG DIRECTION: a Dpad jog moved measuredAngleDeg the WRONG way"
                    + " (Left must make it go UP). Servo and encoder directions disagree; see"
                    + " ENCODER_REVERSED and SERVO_*_REVERSED in RobotConstants.");
        }
        telemetry.addData("Turret encoder", "port '%s', %s",
                RobotConstants.Turret.ENCODER_PORT,
                encoderSeenMoving ? "counting" : "count has NOT changed since Init");

        String aimingAt;
        if (!enabled || mode == TurretMode.DISABLED) {
            aimingAt = "NOTHING (stopped)";
        } else if (mode == TurretMode.MANUAL) {
            aimingAt = "NOTHING (manual)";
        } else if (encoderFault) {
            aimingAt = "NOTHING (stopped by fault)";
        } else if (mode == TurretMode.HOME) {
            aimingAt = String.format("FIXED ANGLE %.0f deg (not the hive)", Math.toDegrees(homeAngleRad));
        } else if (autoSelectSide) {
            aimingAt = aimingAtTopSide ? "HIVE TOP" : "HIVE BOTTOM";
        } else {
            aimingAt = "FIXED POINT";
        }
        telemetry.addData("Turret aiming at", aimingAt);
        telemetry.addData("Turret measuredAngleDeg", "%.1f", getCurrentAngleDegrees());
        telemetry.addData("Turret relativeTargetDeg", "%.1f", getTargetAngleDegrees());
        telemetry.addData("Turret angleErrorDeg", "%.2f (onTarget=%b)", getAngleErrorDegrees(), isOnTarget());
        telemetry.addData("Turret lastCommandedPower", "%.2f", lastCommandedPower);
        telemetry.addData("Turret encoderTicks", lastEncoderTicks);
        telemetry.addData("Turret targetGlobalDeg", "%.1f", getTargetGlobalDegrees());
        telemetry.addData("Turret aimed at target", isAimedAtTarget());
    }

    /**
     * @return true if the last {@link #update(Pose)} measured the turret within tolerance
     *         of the angle it was told to hold. In HOME mode that angle is the home angle,
     *         NOT the hive; use {@link #isAimedAtTarget()} to decide whether to shoot.
     *         False before the first update, whenever the turret is stopped, and from a
     *         target change until the next update.
     */
    public boolean isOnTarget() {
        return onTarget;
    }

    /**
     * @return true only if the turret is TRACKING the field target, the last
     *         {@link #update(Pose)} measured it within tolerance. This is the
     *         "safe to shoot" check.
     */
    public boolean isAimedAtTarget() {
        return enabled && !encoderFault && mode == TurretMode.TRACKING && onTarget;
    }
}
