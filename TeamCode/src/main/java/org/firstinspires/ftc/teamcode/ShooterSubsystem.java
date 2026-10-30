package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Single-motor flywheel shooter with velocity control.
 *
 * <p>OpModes set a target with {@link #setTargetRpm(double)} or
 * {@link #setTargetFromDistance(double)}, then call {@link #update()} once per
 * loop. Like the intake, {@link #stop()} only changes state; {@code update()}
 * writes the motor.
 *
 * <p>Hardware: one {@link DcMotorEx} with an encoder named
 * {@link RobotConstants.Shooter#MOTOR}.
 */
public class ShooterSubsystem {

    private final DcMotorEx motor;
    private double targetRpm;

    /**
     * Finds the shooter motor and puts it in velocity mode.
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.MOTOR);

        // Float so the flywheel spins down instead of slamming to a stop.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // TODO(team): verify direction — FORWARD means positive RPM shoots out
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // RUN_USING_ENCODER lets setVelocity() hold RPM with the hub's PIDF.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetRpm = 0.0;
        motor.setPower(0.0);
    }

    /**
     * Requests a flywheel speed. Takes effect on the next {@link #update()}.
     *
     * @param rpm target output RPM; 0 stops the flywheel
     */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    /**
     * Requests the RPM the tuning table gives for this distance.
     *
     * @param distanceInches robot center to hive center, inches
     */
    public void setTargetFromDistance(double distanceInches) {
        setTargetRpm(rpmForDistance(distanceInches));
    }

    /**
     * Spins the flywheel down. Always safe to call.
     */
    public void stop() {
        targetRpm = 0.0;
    }

    /**
     * Writes the motor velocity for the current target. Call once per loop.
     */
    public void update() {
        if (targetRpm <= 0.0) {
            motor.setPower(0.0);
        } else {
            motor.setVelocity(targetRpm * RobotConstants.Shooter.TICKS_PER_REV / 60.0);
        }
    }

    /**
     * @return the requested RPM
     */
    public double getTargetRpm() {
        return targetRpm;
    }

    /**
     * @return measured flywheel RPM from the encoder
     */
    public double getCurrentRpm() {
        return motor.getVelocity() * 60.0 / RobotConstants.Shooter.TICKS_PER_REV;
    }

    /**
     * @return {@code true} if spinning and within
     *         {@link RobotConstants.Shooter#RPM_TOLERANCE} of target
     */
    public boolean atSpeed() {
        return targetRpm > 0.0
                && Math.abs(getCurrentRpm() - targetRpm) <= RobotConstants.Shooter.RPM_TOLERANCE;
    }

    /**
     * Linearly interpolates {@link RobotConstants.Shooter#RPMS} over
     * {@link RobotConstants.Shooter#DISTANCES_INCHES}, clamped at both ends.
     *
     * @param distanceInches robot center to hive center, inches
     * @return target RPM for that distance
     */
    public static double rpmForDistance(double distanceInches) {
        double[] d = RobotConstants.Shooter.DISTANCES_INCHES;
        double[] rpm = RobotConstants.Shooter.RPMS;

        if (distanceInches <= d[0]) {
            return rpm[0];
        }
        for (int i = 1; i < d.length; i++) {
            if (distanceInches <= d[i]) {
                double t = (distanceInches - d[i - 1]) / (d[i] - d[i - 1]);
                return rpm[i - 1] + t * (rpm[i] - rpm[i - 1]);
            }
        }
        return rpm[rpm.length - 1];
    }
}
