package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric turret driven by TWO continuous-rotation servos, continuously
 * aiming at a field coordinate (the hive) using the Pedro Pathing pose.
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>Two {@link CRServo}s ({@code SERVO_LEFT}, {@code SERVO_RIGHT}) that BOTH push the
 *       turret in the same direction (set the reversed flags in RobotConstants).</li>
 *   <li>NO encoder. The turret angle is an ESTIMATE: commanded power x time x a calibrated
 *       speed. It can drift, so the turret must start pointing straight forward, and
 *       {@link #resetAngle()} re-zeroes the estimate whenever it is physically at home.</li>
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
    }

    private final CRServo leftServo;
    private final CRServo rightServo;
    private final ElapsedTime estimateTimer = new ElapsedTime();
    private final PIDFController pidfController;
    private final ElapsedTime targetVelTimer = new ElapsedTime();

    /** High-level turret operating mode. */
    public enum TurretMode {
        /** Continuously aims at the field target coordinate. */
        TRACKING,
        /** Returns to and holds {@link RobotConstants.Turret#HOME_ANGLE_RADIANS}. */
        HOME,
        /** Servos unpowered. */
        DISABLED
    }

    private TurretMode mode = TurretMode.TRACKING;

    // Geometry & servo speed model
    private double xOffsetInches = RobotConstants.Turret.OFFSET_X_INCHES;
    private double yOffsetInches = RobotConstants.Turret.OFFSET_Y_INCHES;
    private double radPerSecAtFullPower = RobotConstants.Turret.RAD_PER_SEC_AT_FULL_POWER;
    private double estimatedAngleRad = 0.0;

    // Rotation soft limits (prevent wire binding)
    private boolean useSoftLimits = RobotConstants.Turret.USE_SOFT_LIMITS;
    private double minAngleRadians = RobotConstants.Turret.MIN_ANGLE_RADIANS;
    private double maxAngleRadians = RobotConstants.Turret.MAX_ANGLE_RADIANS;

    // Field target (hive) in Pedro Pathing inches
    private double targetXInches = RobotConstants.Turret.DEFAULT_TARGET_X_INCHES;
    private double targetYInches = RobotConstants.Turret.DEFAULT_TARGET_Y_INCHES;

    // The hive has two sides. By default the side is picked from the robot's Y position.
    private boolean autoSelectSide = true;
    private boolean aimingAtTopSide = true;

    private double kV = RobotConstants.Turret.KV;

    // Internal tracking state
    private double previousTargetAngleRad = 0.0;
    private boolean firstTargetSample = true;
    private boolean enabled = true;

    private double currentAngleRad = 0.0;
    private double relativeTargetAngleRad = 0.0;
    private double angleErrorRad = 0.0;
    private double lastCommandedPower = 0.0;

    /**
     * Initializes both continuous servos from the hardware map.
     * The angle estimate starts at 0 here, so the turret must be pointing straight
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

    /** Declares the turret's current position to be 0 rad (pointing forward). Call only when it really is. */
    public void resetAngle() {
        estimatedAngleRad = 0.0;
        estimateTimer.reset();
    }

    /** Integrates the last commanded power over the elapsed time to estimate the turret angle. */
    private void updateEstimate() {
        double dt = Range.clip(estimateTimer.seconds(), 0.0, 0.1);
        estimateTimer.reset();
        estimatedAngleRad += lastCommandedPower * radPerSecAtFullPower * dt;
    }

    /**
     * Call once per loop. Computes the chassis-relative angle to the target from the
     * Pedro pose and drives the servos with PIDF control.
     *
     * @param currentPose current robot {@link Pose} from Pedro Pathing
     */
    public void update(Pose currentPose) {
        updateEstimate();

        if (!enabled || mode == TurretMode.DISABLED) {
            stop();
            return;
        }

        if (mode == TurretMode.HOME) {
            driveToRelativeAngle(RobotConstants.Turret.HOME_ANGLE_RADIANS);
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
        double targetGlobalAngle = Math.atan2(targetYInches - turretWorldY, targetXInches - turretWorldX);
        double relativeTargetAngle = AngleUnit.normalizeRadians(targetGlobalAngle - robotHeading);

        driveToRelativeAngle(relativeTargetAngle);
    }

    private void driveToRelativeAngle(double relativeTargetAngle) {
        if (useSoftLimits) {
            relativeTargetAngle = Range.clip(relativeTargetAngle, minAngleRadians, maxAngleRadians);
        }
        relativeTargetAngleRad = relativeTargetAngle;

        // Target angular velocity (rad/s) for optional kV feedforward
        double dt = targetVelTimer.seconds();
        targetVelTimer.reset();
        double targetAngularVelocityRadSec = 0.0;
        if (!firstTargetSample && dt > 1e-4 && dt <= 0.5) {
            targetAngularVelocityRadSec =
                    AngleUnit.normalizeRadians(relativeTargetAngleRad - previousTargetAngleRad) / dt;
        } else {
            firstTargetSample = false;
        }
        previousTargetAngleRad = relativeTargetAngleRad;

        // Current angle = dead-reckoned estimate (no encoder)
        currentAngleRad = estimatedAngleRad;
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

        if (Math.abs(angleErrorRad) <= RobotConstants.Turret.ANGLE_TOLERANCE_RADIANS
                && Math.abs(targetAngularVelocityRadSec) < 0.05) {
            pidfController.reset();
            setServoPower(0.0);
            return;
        }

        double rawPower = pidfController.calculate(angleErrorRad, targetAngularVelocityRadSec, kV);

        // Hard cutoff if already at/beyond a soft limit and still pushing outward
        if (useSoftLimits) {
            if ((currentAngleRad >= maxAngleRadians && rawPower > 0.0)
                    || (currentAngleRad <= minAngleRadians && rawPower < 0.0)) {
                rawPower = 0.0;
            }
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

    /** Stops both servos and resets PIDF state. */
    public void stop() {
        pidfController.reset();
        firstTargetSample = true;
        setServoPower(0.0);
    }

    public double getCurrentAngleRadians() {
        return estimatedAngleRad;
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

    public boolean isOnTarget() {
        return Math.abs(angleErrorRad) <= RobotConstants.Turret.ANGLE_TOLERANCE_RADIANS;
    }
}
