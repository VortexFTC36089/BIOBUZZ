package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Dual-motor outtake subsystem using encoder velocity control (RPM).
 *
 * <p>OpModes should call {@link #outtake()} (or {@link #setTargetRpm(double)})
 * or {@link #stopMotor()} / {@link #stop()}, then call {@link #update()} once
 * per loop so hardware outputs always match the requested state. The outtake
 * motors are one-way only and cannot reverse.
 *
 * <p>Hardware: two {@link DcMotorEx} motors with encoders named
 * {@link RobotConstants.Outtake#LEFT_MOTOR} and
 * {@link RobotConstants.Outtake#RIGHT_MOTOR}.
 */
public class OuttakeSubsystem {

    /**
     * What the outtake motors are doing this loop.
     */
    public enum MotorState {
        OFF,
        OUTTAKING
    }

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private MotorState motorState;
    private double targetRpm;

    /**
     * Finds both outtake motors and puts them in a known-safe initial state
     * (stopped in velocity mode with PIDF gains applied).
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     */
    public OuttakeSubsystem(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.LEFT_MOTOR);
        rightMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.RIGHT_MOTOR);

        // Float so the flywheels spin down smoothly instead of slamming to a stop.
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Dual opposing wheels typically spin opposite motor directions so both
        // push the game piece forward.
        // TODO(team): verify directions — positive RPM on both should score/eject
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // RUN_USING_ENCODER lets setVelocity() hold target RPM with the hub's PIDF.
        configureVelocityControl(leftMotor);
        configureVelocityControl(rightMotor);

        motorState = MotorState.OFF;
        targetRpm = 0.0;

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    private void configureVelocityControl(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(
                RobotConstants.Outtake.KP,
                RobotConstants.Outtake.KI,
                RobotConstants.Outtake.KD,
                RobotConstants.Outtake.KF
        );
    }

    /**
     * Spins both outtake motors up to {@link RobotConstants.Outtake#TARGET_RPM}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void outtake() {
        setTargetRpm(RobotConstants.Outtake.TARGET_RPM);
    }

    /**
     * Spins both outtake motors up to a specific non-negative RPM. Negative
     * values are clamped to {@code 0.0} so the outtake never reverses.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     *
     * @param rpm target output RPM (positive to outtake, {@code <= 0} to stop)
     */
    public void setTargetRpm(double rpm) {
        if (rpm > 0.0) {
            targetRpm = rpm;
            motorState = MotorState.OUTTAKING;
        } else {
            targetRpm = 0.0;
            motorState = MotorState.OFF;
        }
    }

    /**
     * Requests zero RPM on both outtake motors.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stopMotor() {
        setTargetRpm(0.0);
    }

    /**
     * Stops both outtake motors. Always safe to call from OpMode {@code stop()}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stop() {
        stopMotor();
    }

    /**
     * Writes motor velocity (ticks/sec) to both outtake motors for the current state.
     *
     * <p>Call once per OpMode loop.</p>
     */
    public void update() {
        if (motorState == MotorState.OFF || targetRpm <= 0.0) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
        } else {
            double ticksPerSec = targetRpm * RobotConstants.Outtake.TICKS_PER_REV / 60.0;
            leftMotor.setVelocity(ticksPerSec);
            rightMotor.setVelocity(ticksPerSec);
        }
    }

    /**
     * @return the requested target RPM
     */
    public double getTargetRpm() {
        return targetRpm;
    }

    /**
     * @return measured RPM of the left outtake motor from its encoder
     */
    public double getLeftRpm() {
        return leftMotor.getVelocity() * 60.0 / RobotConstants.Outtake.TICKS_PER_REV;
    }

    /**
     * @return measured RPM of the right outtake motor from its encoder
     */
    public double getRightRpm() {
        return rightMotor.getVelocity() * 60.0 / RobotConstants.Outtake.TICKS_PER_REV;
    }

    /**
     * @return average measured outtake RPM across both motors
     */
    public double getCurrentRpm() {
        return (getLeftRpm() + getRightRpm()) / 2.0;
    }

    /**
     * @return {@code true} if spinning and both motors are within
     *         {@link RobotConstants.Outtake#RPM_TOLERANCE} of {@link #getTargetRpm()}
     */
    public boolean atSpeed() {
        return targetRpm > 0.0
                && Math.abs(getLeftRpm() - targetRpm) <= RobotConstants.Outtake.RPM_TOLERANCE
                && Math.abs(getRightRpm() - targetRpm) <= RobotConstants.Outtake.RPM_TOLERANCE;
    }

    /**
     * @return current requested motor state ({@code OFF} or {@code OUTTAKING})
     */
    public MotorState getMotorState() {
        return motorState;
    }
}
