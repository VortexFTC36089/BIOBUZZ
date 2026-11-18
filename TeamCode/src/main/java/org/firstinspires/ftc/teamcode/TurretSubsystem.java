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
 *   <li>A REV Through Bore Encoder V2 on one servo's output shaft. Its cable is plugged
 *       into the encoder socket of an existing motor port ({@code ENCODER_PORT}), and the
 *       count is read through that motor. This class only READS that motor's encoder; it
 *       never changes the motor's mode, power or direction, and the motor's own
 *       direction setting does not affect the turret angle.</li>
 * </ul>
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
        /** Operator is jogging the turret with a fixed power ({@link #jog(double)}). */
        MANUAL,
        /** Servos commanded to zero power (stopped). */
        DISABLED
    }

    /** Power applied while in {@link TurretMode#MANUAL} (+ = counterclockwise). */
    private double manualPower = 0.0;

    // ---- Direction / encoder monitor ----
    // Runtime servo sign (+1 or -1). Flipped when the encoder moves AGAINST a sustained
    // commanded power: that can only mean the servo direction flags are wrong. Keeps a
    // wrong flag from turning the control loop into a runaway.
    private int servoSign = 1;
    private int autoFlipCount = 0;
    /** True once the servos were driven hard for a while and the encoder never moved. */
    private boolean encoderFault = false;
    private double monitorPowerSign = 0.0;
    private double monitorMinAbsPower = 0.0;
    private double monitorStartAngleRad = 0.0;
    private int monitorStartTicks = 0;
    private final ElapsedTime monitorTimer = new ElapsedTime();

    private TurretMode mode = TurretMode.TRACKING;

    /** Chassis-relative angle (radians) held while in {@link TurretMode#HOME}. */
    private double homeAngleRad = RobotConstants.Turret.HOME_ANGLE_RADIANS;

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
    private double minAngleRadians = RobotConstants.Turret.MIN_ANGLE_RADIANS;
    private double maxAngleRadians = RobotConstants.Turret.MAX_ANGLE_RADIANS;

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
     * The result is continuous (it keeps counting past +/- PI), which is what the
     * soft limits need; {@link #driveToRelativeAngle} wraps the ERROR, not the angle.
     */
    private double readAngleRadians() {
        int rawTicks = readRawTicks();
        lastEncoderTicks = rawTicks;
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
        currentAngleRad = readAngleRadians();
        monitorDirection();

        if (!enabled || mode == TurretMode.DISABLED || encoderFault) {
            stop();
            return;
        }

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

        // Don't let the shortest path cross the mechanical limits (cable wrap)
        if (useSoftLimits) {
            double unwrappedTarget = currentAngleRad + angleErrorRad;
            if (unwrappedTarget > maxAngleRadians) {
                angleErrorRad -= 2.0 * Math.PI;
            } else if (unwrappedTarget < minAngleRadians) {
                angleErrorRad += 2.0 * Math.PI;
            }
        }

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
     * Watches the measured angle against the power commanded on the previous loop.
     *
     * <ul>
     *   <li>Encoder moving the WRONG way by {@link RobotConstants.Turret#RUNAWAY_DEGREES}
     *       while the power stays at or above
     *       {@link RobotConstants.Turret#DIRECTION_MONITOR_MIN_POWER} in one direction:
     *       the servo direction flags are wrong. Flip the runtime servo sign so the loop
     *       is stable again, and report it so the flags get fixed in RobotConstants.</li>
     *   <li>Encoder count not changing at all for
     *       {@link RobotConstants.Turret#ENCODER_STALL_SECONDS} while the power stays at
     *       or above {@link RobotConstants.Turret#ENCODER_STALL_MIN_POWER}: the turret is
     *       not being measured (or cannot move). Latch {@link #encoderFault}, which stops
     *       the servos until {@link #clearFault()}.</li>
     * </ul>
     */
    private void monitorDirection() {
        double p = lastCommandedPower;
        double sign = Math.abs(p) >= RobotConstants.Turret.DIRECTION_MONITOR_MIN_POWER
                ? Math.signum(p) : 0.0;
        if (sign == 0.0 || sign != monitorPowerSign) {
            restartMonitorWindow(sign, Math.abs(p));
            return;
        }
        monitorMinAbsPower = Math.min(monitorMinAbsPower, Math.abs(p));

        double movedRad = currentAngleRad - monitorStartAngleRad; // continuous, no wrap
        double runawayRad = Math.toRadians(RobotConstants.Turret.RUNAWAY_DEGREES);

        if (sign * movedRad <= -runawayRad) {
            // Pushed one way, went the other way: servo direction is backwards.
            servoSign = -servoSign;
            autoFlipCount++;
            pidfController.reset();
            restartMonitorWindow(0.0, 0.0);
        } else if (sign * movedRad >= runawayRad) {
            // Moving the right way. Start a fresh window from here.
            restartMonitorWindow(sign, Math.abs(p));
        } else if (monitorTimer.seconds() >= RobotConstants.Turret.ENCODER_STALL_SECONDS
                && lastEncoderTicks == monitorStartTicks
                && monitorMinAbsPower >= RobotConstants.Turret.ENCODER_STALL_MIN_POWER) {
            encoderFault = true;
            restartMonitorWindow(0.0, 0.0);
        }
    }

    private void restartMonitorWindow(double powerSign, double absPower) {
        monitorPowerSign = powerSign;
        monitorMinAbsPower = absPower;
        monitorStartAngleRad = currentAngleRad;
        monitorStartTicks = lastEncoderTicks;
        monitorTimer.reset();
    }

    /**
     * Operator jog: drive the turret with a fixed power until {@link #holdCurrentAngle()}
     * (or another mode) is called. Switches to {@link TurretMode#MANUAL}.
     *
     * @param power servo power, + = counterclockwise (left from above)
     */
    public void jog(double power) {
        if (mode != TurretMode.MANUAL) {
            setMode(TurretMode.MANUAL);
        }
        manualPower = power;
    }

    /**
     * Stops jogging and holds the turret where it is now ({@link TurretMode#HOME} at the
     * current measured angle). Used when the jog button is released.
     */
    public void holdCurrentAngle() {
        manualPower = 0.0;
        homeAngleRad = currentAngleRad;
        setMode(TurretMode.HOME);
        // setMode() only resets when the mode changes; make sure the hold starts clean.
        firstTargetSample = true;
        onTarget = false;
    }

    /** @return true if the servos are stopped because the encoder never counted (see {@link #clearFault()}) */
    public boolean hasEncoderFault() {
        return encoderFault;
    }

    /** Clears {@link #hasEncoderFault()} so the turret is driven again. */
    public void clearFault() {
        encoderFault = false;
        restartMonitorWindow(0.0, 0.0);
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
     * Adds the turret aiming diagnostics to the Driver Station. The caller still calls
     * {@code telemetry.update()}.
     *
     * <ul>
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
        if (encoderFault) {
            telemetry.addLine("!! TURRET ENCODER NOT COUNTING: servos STOPPED. The turret was driven"
                    + " hard and the encoder ticks never changed. Check the encoder cable in the"
                    + " encoder socket of the intake motor port (and that the servos move)."
                    + " G2 Dpad Down retries.");
        }
        if (autoFlipCount > 0) {
            telemetry.addLine(String.format(
                    "!! SERVO DIRECTION AUTO-FLIPPED (%d x, now %s): set SERVO_LEFT_REVERSED and"
                            + " SERVO_RIGHT_REVERSED to %b in RobotConstants.",
                    autoFlipCount, servoSign < 0 ? "reversed" : "as configured",
                    RobotConstants.Turret.SERVO_LEFT_REVERSED != (servoSign < 0)));
        }
        String aimingAt;
        if (!enabled || mode == TurretMode.DISABLED || encoderFault) {
            aimingAt = "NOTHING (stopped)";
        } else if (mode == TurretMode.MANUAL) {
            aimingAt = String.format("MANUAL JOG power %.2f", manualPower);
        } else if (mode == TurretMode.HOME) {
            aimingAt = String.format("FIXED ANGLE %.0f deg (not the hive)", Math.toDegrees(homeAngleRad));
        } else if (autoSelectSide) {
            aimingAt = aimingAtTopSide ? "HIVE TOP" : "HIVE BOTTOM";
        } else {
            aimingAt = "FIXED POINT";
        }
        telemetry.addData("Turret aiming at", aimingAt);
        telemetry.addData("Turret targetGlobalDeg", "%.1f", getTargetGlobalDegrees());
        telemetry.addData("Turret relativeTargetDeg", "%.1f", getTargetAngleDegrees());
        telemetry.addData("Turret measuredAngleDeg", "%.1f", getCurrentAngleDegrees());
        telemetry.addData("Turret encoderTicks", lastEncoderTicks);
        telemetry.addData("Turret angleErrorDeg", "%.2f (onTarget=%b)", getAngleErrorDegrees(), isOnTarget());
        telemetry.addData("Turret aimed at target", isAimedAtTarget());
        telemetry.addData("Turret lastCommandedPower", "%.2f", lastCommandedPower);
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
     *         {@link #update(Pose)} measured it within tolerance, and the aim angle did
     *         not have to be clipped to a soft limit. This is the "safe to shoot" check.
     */
    public boolean isAimedAtTarget() {
        return enabled && !encoderFault && mode == TurretMode.TRACKING && onTarget && !targetClipped;
    }
}
