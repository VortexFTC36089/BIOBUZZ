package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Outtake subsystem consisting of a velocity-controlled outtake motor (RPM).
 *
 * <p>OpModes should call {@link #outtake()} (or {@link #setTargetRpm(double)})
 * or {@link #stopMotor()} / {@link #stop()}, then call {@link #update()} once
 * per loop so hardware outputs always match the requested state. The outtake
 * motor is one-way only and cannot reverse.
 *
 * <p>Hardware: one {@link DcMotorEx} with an encoder named
 * {@link RobotConstants.Outtake#MOTOR}.
 */
public class OuttakeSubsystem {

    /**
     * What the outtake motor is doing this loop.
     */
    public enum MotorState {
        OFF,
        OUTTAKING
    }

    private final DcMotorEx motor;

    private MotorState motorState;
    private double targetRpm;

    /**
     * Finds the outtake motor and puts it in a known-safe initial state
     * (stopped in velocity mode).
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     */
    public OuttakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.MOTOR);

        // Float so the flywheel spins down smoothly instead of slamming to a stop.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // TODO(team): verify direction — FORWARD means positive RPM scores/ejects
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // RUN_USING_ENCODER lets setVelocity() hold target RPM with the hub's PIDF.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(
                RobotConstants.Outtake.KP,
                RobotConstants.Outtake.KI,
                RobotConstants.Outtake.KD,
                RobotConstants.Outtake.KF
        );

        motorState = MotorState.OFF;
        targetRpm = 0.0;

        motor.setPower(0.0);
    }

    /**
     * Spins the outtake motor up to {@link RobotConstants.Outtake#TARGET_RPM}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void outtake() {
        setTargetRpm(RobotConstants.Outtake.TARGET_RPM);
    }

    /**
     * Spins the outtake motor up to a specific non-negative RPM. Negative
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
     * Requests zero RPM on the outtake motor.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stopMotor() {
        setTargetRpm(0.0);
    }

    /**
     * Stops the outtake motor. Always safe to call from OpMode {@code stop()}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stop() {
        stopMotor();
    }

    /**
     * Writes motor velocity (ticks/sec) for the current state.
     *
     * <p>Call once per OpMode loop.</p>
     */
    public void update() {
        if (motorState == MotorState.OFF || targetRpm <= 0.0) {
            motor.setPower(0.0);
        } else {
            motor.setVelocity(targetRpm * RobotConstants.Outtake.TICKS_PER_REV / 60.0);
        }
    }

    /**
     * @return the requested target RPM
     */
    public double getTargetRpm() {
        return targetRpm;
    }

    /**
     * @return measured outtake RPM from the encoder
     */
    public double getCurrentRpm() {
        return motor.getVelocity() * 60.0 / RobotConstants.Outtake.TICKS_PER_REV;
    }

    /**
     * @return {@code true} if spinning and within
     *         {@link RobotConstants.Outtake#RPM_TOLERANCE} of {@link #getTargetRpm()}
     */
    public boolean atSpeed() {
        return targetRpm > 0.0
                && Math.abs(getCurrentRpm() - targetRpm) <= RobotConstants.Outtake.RPM_TOLERANCE;
    }

    /**
     * @return current requested motor state ({@code OFF} or {@code OUTTAKING})
     */
    public MotorState getMotorState() {
        return motorState;
    }
}
