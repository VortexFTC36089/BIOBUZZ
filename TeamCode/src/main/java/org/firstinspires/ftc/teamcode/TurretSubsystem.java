package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 *   <li>A REV Through Bore Encoder V2 on {@code ENCODER_PORT}. Angle is read from that
 *       encoder, so the turret must start pointing straight forward and
 *       {@link #resetAngle()} re-zeroes the reading whenever it is physically at home.
 *       Rotation is continuous: the shortest path across +/- PI is always used.</li>
 * </ul>
 */
public class TurretSubsystem {

    /**
     * PIDF controller: proportional, integral, derivative, velocity feedforward (kV)
     * and static feedforward (kF, breakaway power).
     */
    public static class PIDFController {
        private double kP;
        private double kI;
        private double kD;
        private double kF;
        private double maxIntegral = RobotConstants.Turret.MAX_INTEGRAL;

        private double integralSum = 0.0;
        private double prevError = 0.0;
        private boolean firstUpdate = true;
        private final ElapsedTime timer = new ElapsedTime();

        public PIDFController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        public double calculate(double angleError, double targetVelocityRadSec, double kV) {
            double dt = timer.seconds();
            timer.reset();

            if (firstUpdate || dt <= 0.0 || dt > 0.5) {
                dt = 0.0;
                firstUpdate = false;
            }

            double pTerm = kP * angleError;

            if (dt > 0.0 && kI != 0.0) {
                integralSum = Range.clip(integralSum + angleError * dt, -maxIntegral, maxIntegral);
            }
            double iTerm = kI * integralSum;

            double derivative = (dt > 0.0) ? (angleError - prevError) / dt : 0.0;
            double dTerm = kD * derivative;
            prevError = angleError;

            double vTerm = kV * targetVelocityRadSec;
            double fTerm = (Math.abs(angleError) > 1e-4) ? Math.signum(angleError) * kF : 0.0;

            return pTerm + iTerm + dTerm + vTerm + fTerm;
        }

        public void setPIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
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
         * Restarts the loop timing but KEEPS the integral. Call while the output is idle
         * (turret aimed) so the next {@link #calculate} does not see a stale time step.
         */
        public void resetTiming() {
            firstUpdate = true;
            timer.reset();
        }
    }

    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx turretEncoder;
    private int zeroOffsetTicks = 0;
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
        /** Servos unpowered. */
        DISABLED
    }

    private TurretMode mode = TurretMode.TRACKING;

    /** Chassis-relative angle (radians) held while in {@link TurretMode#HOME}. */
    private double homeAngleRad = RobotConstants.Turret.HOME_ANGLE_RADIANS;

    // Geometry
    private double xOffsetInches = RobotConstants.Turret.OFFSET_X_INCHES;
    private double yOffsetInches = RobotConstants.Turret.OFFSET_Y_INCHES;

    // Field target (hive) in Pedro Pathing inches
    private double targetXInches = RobotConstants.Turret.DEFAULT_TARGET_X_INCHES;
    private double targetYInches = RobotConstants.Turret.DEFAULT_TARGET_Y_INCHES;

    // The hive has two sides. By default the side is picked from the robot's Y position.
    private boolean autoSelectSide = true;
    private boolean aimingAtTopSide = true;

    private double kV = RobotConstants.Turret.KV;

    // Internal tracking state
    private double previousTargetAngleRad = 0.0;
    private double targetAngularVelocityRadSec = 0.0; // low-pass filtered
    private boolean firstTargetSample = true;
    private boolean enabled = true;

    private double currentAngleRad = 0.0;
    private double targetGlobalAngleRad = 0.0;
    private double relativeTargetAngleRad = 0.0;
    private double angleErrorRad = 0.0;
    private double lastCommandedPower = 0.0;

    /**
     * Initializes both continuous servos and the through-bore encoder.
     * The angle reads 0 here, so the turret must be pointing straight
     * forward (home) when the OpMode is initialized.
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, RobotConstants.Turret.SERVO_LEFT);
        rightServo = hardwareMap.get(CRServo.class, RobotConstants.Turret.SERVO_RIGHT);

        // Both servos must push the turret the SAME way, otherwise they fight each other.
        leftServo.setDirection(RobotConstants.Turret.SERVO_LEFT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightServo.setDirection(RobotConstants.Turret.SERVO_RIGHT_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        turretEncoder = hardwareMap.get(DcMotorEx.class, RobotConstants.Turret.ENCODER_PORT);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetAngle();

        pidfController = new PIDFController(
                RobotConstants.Turret.KP,
                RobotConstants.Turret.KI,
                RobotConstants.Turret.KD,
                RobotConstants.Turret.KF
        );
        targetVelTimer.reset();
        setServoPower(0.0);
    }

    /**
     * Declares the turret's current position to be 0 rad (pointing forward).
     * Call only when it really is. Stores an offset instead of resetting the
     * hub encoder mode, so this is safe to call mid-match.
     */
    public void resetAngle() {
        zeroOffsetTicks = turretEncoder.getCurrentPosition();
    }

    /**
     * Call once per loop. Computes the chassis-relative angle to the target from the
     * Pedro pose and drives the servos with PIDF control.
     *
     * @param currentPose current robot {@link Pose} from Pedro Pathing
     */
    public void update(Pose currentPose) {
        if (!enabled || mode == TurretMode.DISABLED) {
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
        relativeTargetAngleRad = relativeTargetAngle;

        // Target angular velocity (rad/s) for kV feedforward. It is capped at the turret's
        // top speed and low-pass filtered, so one jittery loop time or a target switch
        // cannot spike the feedforward.
        double dt = targetVelTimer.seconds();
        targetVelTimer.reset();
        double maxVelocity = RobotConstants.Turret.MAX_TARGET_VELOCITY_RAD_SEC;
        if (!firstTargetSample && dt > 1e-4 && dt <= 0.5) {
            double rawVelocity = Range.clip(
                    AngleUnit.normalizeRadians(relativeTargetAngleRad - previousTargetAngleRad) / dt,
                    -maxVelocity, maxVelocity);
            double alpha = dt / (RobotConstants.Turret.TARGET_VELOCITY_FILTER_SECONDS + dt);
            targetAngularVelocityRadSec += alpha * (rawVelocity - targetAngularVelocityRadSec);
        } else {
            firstTargetSample = false;
            targetAngularVelocityRadSec = 0.0;
        }
        previousTargetAngleRad = relativeTargetAngleRad;

        currentAngleRad = getCurrentAngleRadians();
        // Shortest path across +/- PI, including after multiple revolutions.
        angleErrorRad = AngleUnit.normalizeRadians(relativeTargetAngleRad - currentAngleRad);

        // Aimed: stop the servos. This depends only on the error, not on how fast the
        // target is moving, and it keeps the integral so a slowly moving target does not
        // restart the controller from zero every time the turret catches up.
        if (Math.abs(angleErrorRad) <= RobotConstants.Turret.ANGLE_TOLERANCE_RADIANS) {
            pidfController.resetTiming();
            setServoPower(0.0);
            return;
        }

        double rawPower = pidfController.calculate(angleErrorRad, targetAngularVelocityRadSec, kV);

        // Outside tolerance, never send a command too weak to move the turret: push
        // toward the target with at least the minimum power that beats stiction.
        double minPower = RobotConstants.Turret.MIN_COMMAND_POWER;
        if (Math.abs(rawPower) < minPower) {
            rawPower = Math.signum(angleErrorRad) * minPower;
        }

        double maxPower = RobotConstants.Turret.MAX_POWER;
        setServoPower(Range.clip(rawPower, -maxPower, maxPower));
    }

    /** Sends the same power to both servos. */
    private void setServoPower(double power) {
        lastCommandedPower = power;
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    /**
     * Manually sets the target coordinate (Pedro inches). This turns OFF automatic hive-side
     * selection; call {@link #setAutoSelectSide(boolean)} with true to turn it back on.
     */
    public void setTargetCoordinate(double x, double y) {
        this.autoSelectSide = false;
        this.targetXInches = x;
        this.targetYInches = y;
    }

    /**
     * Picks the upper or lower hive side from the robot's Y position. A small hysteresis band
     * around the midline stops the target from flipping back and forth when the robot is
     * driving along the line.
     */
    private void selectTargetSide(Pose pose) {
        if (!autoSelectSide || pose == null) {
            return;
        }
        double y = pose.y();
        double mid = RobotConstants.Turret.SIDE_SWITCH_Y_INCHES;
        double band = RobotConstants.Turret.SIDE_HYSTERESIS_INCHES;
        if (y > mid + band) {
            aimingAtTopSide = true;
        } else if (y < mid - band) {
            aimingAtTopSide = false;
        }
        targetXInches = aimingAtTopSide
                ? RobotConstants.Turret.TOP_TARGET_X_INCHES : RobotConstants.Turret.BOTTOM_TARGET_X_INCHES;
        targetYInches = aimingAtTopSide
                ? RobotConstants.Turret.TOP_TARGET_Y_INCHES : RobotConstants.Turret.BOTTOM_TARGET_Y_INCHES;
    }

    /** Turns automatic upper/lower hive side selection on or off. */
    public void setAutoSelectSide(boolean autoSelectSide) {
        this.autoSelectSide = autoSelectSide;
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
        this.homeAngleRad = angleRadians;
    }

    public TurretMode getMode() {
        return mode;
    }

    /**
     * Distance in inches from the turret pivot to the hive.
     */
    public double getDistanceToTarget(Pose currentPose) {
        if (currentPose == null) {
            return 0.0;
        }
        selectTargetSide(currentPose);
        double heading = currentPose.heading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double turretWorldX = currentPose.x() + (xOffsetInches * cosH - yOffsetInches * sinH);
        double turretWorldY = currentPose.y() + (xOffsetInches * sinH + yOffsetInches * cosH);

        return Math.hypot(targetXInches - turretWorldX, targetYInches - turretWorldY);
    }

    public void setPIDFGains(double kP, double kI, double kD, double kV, double kF) {
        this.kV = kV;
        this.pidfController.setPIDF(kP, kI, kD, kF);
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

    /** Stops both servos and resets PIDF state. */
    public void stop() {
        pidfController.reset();
        firstTargetSample = true;
        setServoPower(0.0);
    }

    public double getCurrentAngleRadians() {
        int ticks = turretEncoder.getCurrentPosition() - zeroOffsetTicks;
        if (RobotConstants.Turret.ENCODER_REVERSED) {
            ticks = -ticks;
        }
        return (ticks / RobotConstants.Turret.ENCODER_TICKS_PER_REV)
                * (2.0 * Math.PI)
                * RobotConstants.Turret.ENCODER_TO_TURRET_RATIO;
    }

    public double getCurrentAngleDegrees() {
        return Math.toDegrees(getCurrentAngleRadians());
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
     *   <li>{@code targetGlobalDeg}: field angle from the turret to the target</li>
     *   <li>{@code relativeTargetDeg}: that angle relative to the chassis (where the turret should point)</li>
     *   <li>{@code currentAngleDeg}: where the through-bore encoder says the turret points</li>
     *   <li>{@code angleErrorDeg}: relative target minus estimate, shortest path</li>
     *   <li>{@code lastCommandedPower}: power sent to both servos this loop</li>
     * </ul>
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret targetGlobalDeg", "%.1f", getTargetGlobalDegrees());
        telemetry.addData("Turret relativeTargetDeg", "%.1f", getTargetAngleDegrees());
        telemetry.addData("Turret currentAngleDeg", "%.1f", getCurrentAngleDegrees());
        telemetry.addData("Turret angleErrorDeg", "%.2f (onTarget=%b)", getAngleErrorDegrees(), isOnTarget());
        telemetry.addData("Turret lastCommandedPower", "%.2f", lastCommandedPower);
    }

    public boolean isOnTarget() {
        return Math.abs(angleErrorRad) <= RobotConstants.Turret.ANGLE_TOLERANCE_RADIANS;
    }
}
