package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric turret subsystem for the goBILDA Gear-Driven Turret
 * (SKU 3208-0004-0001) driven by a 5203 Series 435 RPM Yellow Jacket Motor,
 * continuously aiming at field coordinates using Pedro Pathing telemetry.
 *
 * <h3>Hardware &amp; Specification Summary</h3>
 * <ul>
 *   <li>Turret: goBILDA Gear-Driven Turret (SKU 3208-0004-0001, 105mm inner bore)</li>
 *   <li>Motor: 5203 Series 435 RPM Yellow Jacket ({@link DcMotorEx} {@code "turretMotor"}, {@code 383.6} ticks/rev)</li>
 *   <li>Gear Reduction: {@code 2.75:1} (64T Hub-Mount Gear) &rarr; {@code 158.18 RPM} ({@code 16.564 rad/s} max)</li>
 *   <li>Total Turret Resolution: {@code 1,054.9 ticks/rev} ({@code 167.89 ticks/rad} | {@code 2.93 ticks/deg})</li>
 *   <li>Control Mode: {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER} with {@link DcMotor.ZeroPowerBehavior#BRAKE}</li>
 *   <li>Closed-Loop Control: Software {@link PIDFController} ({@code kP = 1.2, kI = 0.0, kD = 0.05, kV = 0.0604})</li>
 * </ul>
 */
public class TurretSubsystem {

    /**
     * FTCLib-style PIDFController supporting proportional ({@code kP}),
     * integral ({@code kI}), derivative ({@code kD}), velocity feedforward
     * ({@code kV}), and static feedforward ({@code kF}) terms.
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

        /**
         * Calculates the control effort for a pre-normalized angular error and
         * target angular velocity.
         *
         * @param angleError           shortest-path angular error in radians
         * @param targetVelocityRadSec target angular velocity in rad/s
         * @param kV                   velocity feedforward gain ({@code 1 / 16.564 = 0.0604})
         * @return unclamped motor command
         */
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

    private final DcMotorEx turretMotor;
    private final PIDFController pidfController;
    private final ElapsedTime targetVelTimer = new ElapsedTime();

    /**
     * High-level turret operating mode.
     */
    public enum TurretMode {
        /** Continuously aims at the field target coordinate. */
        TRACKING,
        /** Returns to and holds {@link RobotConstants.Turret#HOME_ANGLE_RADIANS}. */
        HOME,
        /** Motor unpowered. */
        DISABLED
    }

    private TurretMode mode = TurretMode.TRACKING;

    // Geometry & encoder resolution (167.89 ticks/rad | 2.93 ticks/deg)
    private double xOffsetInches = RobotConstants.Turret.OFFSET_X_INCHES;
    private double yOffsetInches = RobotConstants.Turret.OFFSET_Y_INCHES;
    private double ticksPerRadian = RobotConstants.Turret.TICKS_PER_RADIAN; // 167.89

    // Rotation soft limits (bounded to prevent wire binding)
    private boolean useSoftLimits = RobotConstants.Turret.USE_SOFT_LIMITS;
    private double minAngleRadians = RobotConstants.Turret.MIN_ANGLE_RADIANS;
    private double maxAngleRadians = RobotConstants.Turret.MAX_ANGLE_RADIANS;

    // Field target coordinates in Pedro Pathing inches
    private double targetXInches = RobotConstants.Turret.DEFAULT_TARGET_X_INCHES;
    private double targetYInches = RobotConstants.Turret.DEFAULT_TARGET_Y_INCHES;

    // Velocity feedforward gain (1 / 16.564 rad/s = 0.0604)
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
     * Initializes {@code "turretMotor"} from the FTC {@link HardwareMap}, sets
     * {@link DcMotor.ZeroPowerBehavior#BRAKE}, resets the encoder to {@code 0}
     * at the forward startup pose, and places the motor in
     * {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER} for software PIDF control.
     *
     * @param hardwareMap FTC {@link HardwareMap} from the active OpMode
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Turret.MOTOR);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO(team): verify motor direction — FORWARD should rotate turret CCW (+angle)
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0.0);

        pidfController = new PIDFController(
                RobotConstants.Turret.KP,
                RobotConstants.Turret.KI,
                RobotConstants.Turret.KD,
                RobotConstants.Turret.KF
        );
        targetVelTimer.reset();
    }

    /**
     * Updates the turret's world position, calculates the chassis-relative
     * target angle, and commands the motor with closed-loop PIDF + velocity
     * feedforward control.
     *
     * <p>Call once per loop in TeleOp or Autonomous.</p>
     *
     * @param currentPose current robot {@link Pose} from Pedro Pathing
     */
    public void update(Pose currentPose) {
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

        // 1. Coordinate Transformation (Robot to World Frame)
        double robotX = currentPose.x();
        double robotY = currentPose.y();
        double robotHeading = currentPose.heading();

        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double turretWorldX = robotX + (xOffsetInches * cosH - yOffsetInches * sinH);
        double turretWorldY = robotY + (xOffsetInches * sinH + yOffsetInches * cosH);

        // 2. Target Calculation (Global -> Chassis-Relative normalized to [-PI, PI])
        double targetGlobalAngle = Math.atan2(targetYInches - turretWorldY, targetXInches - turretWorldX);
        double relativeTargetAngle = AngleUnit.normalizeRadians(targetGlobalAngle - robotHeading);

        driveToRelativeAngle(relativeTargetAngle);
    }

    /**
     * Runs one PIDF iteration toward a chassis-relative target angle.
     *
     * @param relativeTargetAngle desired turret angle relative to robot forward (radians)
     */
    private void driveToRelativeAngle(double relativeTargetAngle) {
        // Enforce soft rotation limits to prevent wire binding
        if (useSoftLimits) {
            relativeTargetAngle = Range.clip(relativeTargetAngle, minAngleRadians, maxAngleRadians);
        }
        relativeTargetAngleRad = relativeTargetAngle;

        // Compute target angular velocity (rad/s) for kV velocity feedforward
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

        // 3. Motor PIDF Control
        currentAngleRad = turretMotor.getCurrentPosition() / ticksPerRadian; // ticks / 167.89
        angleErrorRad = AngleUnit.normalizeRadians(relativeTargetAngleRad - currentAngleRad);

        // Prevent shortest-path wraparound from crossing the mechanical soft limits
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
            lastCommandedPower = 0.0;
            turretMotor.setPower(0.0);
            return;
        }

        double rawPower = pidfController.calculate(angleErrorRad, targetAngularVelocityRadSec, kV);

        // Hard safety cutoff if already at/beyond soft limits
        if (useSoftLimits) {
            if ((currentAngleRad >= maxAngleRadians && rawPower > 0.0)
                    || (currentAngleRad <= minAngleRadians && rawPower < 0.0)) {
                rawPower = 0.0;
            }
        }

        lastCommandedPower = Range.clip(rawPower, -1.0, 1.0);
        turretMotor.setPower(lastCommandedPower);
    }

    /**
     * Dynamically sets the field target coordinate (in Pedro Pathing inches).
     *
     * @param x target X coordinate in inches
     * @param y target Y coordinate in inches
     */
    public void setTargetCoordinate(double x, double y) {
        this.targetXInches = x;
        this.targetYInches = y;
    }

    /**
     * @return active field target X coordinate in inches
     */
    public double getTargetX() {
        return targetXInches;
    }

    /**
     * @return active field target Y coordinate in inches
     */
    public double getTargetY() {
        return targetYInches;
    }

    /**
     * Sets the turret operating mode. {@link TurretMode#HOME} drives the turret
     * back to {@link RobotConstants.Turret#HOME_ANGLE_RADIANS} and holds it
     * there; {@link TurretMode#TRACKING} resumes field-target aiming.
     *
     * @param mode desired operating mode
     */
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
     * @return current operating mode
     */
    public TurretMode getMode() {
        return mode;
    }

    /**
     * Computes the Euclidean distance (in inches) from the turret's true world
     * position to the field target for shooter flywheel velocity scaling.
     *
     * @param currentPose current robot {@link Pose} from Pedro Pathing
     * @return distance from turret pivot to target in inches
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
     * Updates the PIDF and velocity feedforward gains at runtime.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kV velocity feedforward gain ({@code 0.0604} for {@code 16.564 rad/s})
     * @param kF static friction feedforward gain
     */
    public void setPIDFGains(double kP, double kI, double kD, double kV, double kF) {
        this.kV = kV;
        this.pidfController.setPIDF(kP, kI, kD, kF);
    }

    /**
     * Configures soft rotation limits in degrees relative to the robot chassis.
     *
     * @param useSoftLimits {@code true} to enforce soft limits
     * @param minDegrees    minimum angle in degrees (e.g., {@code -180.0})
     * @param maxDegrees    maximum angle in degrees (e.g., {@code 180.0})
     */
    public void setSoftLimitsDegrees(boolean useSoftLimits, double minDegrees, double maxDegrees) {
        this.useSoftLimits = useSoftLimits;
        this.minAngleRadians = Math.toRadians(minDegrees);
        this.maxAngleRadians = Math.toRadians(maxDegrees);
    }

    /**
     * Configures the mounting offset of the turret pivot from the robot odometry origin.
     *
     * @param xOffsetInches forward offset in inches (+X forward)
     * @param yOffsetInches lateral offset in inches (+Y left)
     */
    public void setMountingOffset(double xOffsetInches, double yOffsetInches) {
        this.xOffsetInches = xOffsetInches;
        this.yOffsetInches = yOffsetInches;
    }

    /**
     * Enables or disables closed-loop turret tracking.
     *
     * @param enabled {@code true} to actively track target, {@code false} to stop
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            stop();
        }
    }

    /**
     * Stops the turret motor and resets PIDF state.
     */
    public void stop() {
        pidfController.reset();
        firstTargetSample = true;
        lastCommandedPower = 0.0;
        turretMotor.setPower(0.0);
    }

    /**
     * @return current turret angle in radians ({@code motor.getCurrentPosition() / 167.89})
     */
    public double getCurrentAngleRadians() {
        return turretMotor.getCurrentPosition() / ticksPerRadian;
    }

    /**
     * @return current turret angle in degrees
     */
    public double getCurrentAngleDegrees() {
        return Math.toDegrees(getCurrentAngleRadians());
    }

    /**
     * @return target turret angle in radians
     */
    public double getTargetAngleRadians() {
        return relativeTargetAngleRad;
    }

    /**
     * @return target turret angle in degrees
     */
    public double getTargetAngleDegrees() {
        return Math.toDegrees(relativeTargetAngleRad);
    }

    /**
     * @return latest shortest-path angular error in radians
     */
    public double getAngleErrorRadians() {
        return angleErrorRad;
    }

    /**
     * @return latest shortest-path angular error in degrees
     */
    public double getAngleErrorDegrees() {
        return Math.toDegrees(angleErrorRad);
    }

    /**
     * @return latest motor power output in {@code [-1.0, 1.0]}
     */
    public double getLastCommandedPower() {
        return lastCommandedPower;
    }

    /**
     * @return {@code true} if the turret is within tolerance of the target angle
     */
    public boolean isOnTarget() {
        return Math.abs(angleErrorRad) <= RobotConstants.Turret.ANGLE_TOLERANCE_RADIANS;
    }
}
