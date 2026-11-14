package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Dual-motor outtake subsystem using encoder velocity control (RPM) with the
 * hub's velocity PIDF.
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
     * What the outtake motors are doing this loop, derived from the measured
     * encoder RPM every {@link #update()}.
     */
    public enum MotorState {
        /** No RPM requested; motors unpowered. */
        OFF,
        /** RPM requested but at least one motor is outside tolerance. */
        SPINNING_UP,
        /** Both motors within {@link RobotConstants.Outtake#RPM_TOLERANCE} of target. */
        AT_SPEED
    }

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private MotorState motorState;
    private double targetRpm;

    // Measured RPM, refreshed once per update() so every check in a loop
    // iteration sees the same encoder sample.
    private double measuredLeftRpm;
    private double measuredRightRpm;

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
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Dual opposing wheels typically spin opposite motor directions so both
        // push the game piece forward.
        // TODO(team): verify directions — positive RPM on both should score/eject
        leftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // RUN_USING_ENCODER lets setVelocity() hold target RPM with the hub's PIDF.
        // Each motor gets its own gains.
        configureVelocityControl(leftMotor,
                RobotConstants.Outtake.LEFT_KP,
                RobotConstants.Outtake.LEFT_KI,
                RobotConstants.Outtake.LEFT_KD,
                RobotConstants.Outtake.LEFT_KF);
        configureVelocityControl(rightMotor,
                RobotConstants.Outtake.RIGHT_KP,
                RobotConstants.Outtake.RIGHT_KI,
                RobotConstants.Outtake.RIGHT_KD,
                RobotConstants.Outtake.RIGHT_KF);

        motorState = MotorState.OFF;
        targetRpm = 0.0;
        measuredLeftRpm = 0.0;
        measuredRightRpm = 0.0;

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    private void configureVelocityControl(DcMotorEx motor, double kP, double kI, double kD, double kF) {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
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
            // Assume we are not yet at speed; update() confirms from encoders.
            if (motorState == MotorState.OFF) {
                motorState = MotorState.SPINNING_UP;
            }
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
     * Reads both encoders, re-commands the target velocity (ticks/sec), and
     * refreshes {@link #getMotorState()} from the measured RPM.
     *
     * <p>Call once per OpMode loop.</p>
     */
    public void update() {
        // 1. Measure: sample both encoders once per loop.
        measuredLeftRpm = ticksPerSecToRpm(leftMotor.getVelocity());
        measuredRightRpm = ticksPerSecToRpm(rightMotor.getVelocity());

        // 2. Command: hold target RPM via the hub's velocity PIDF, or coast.
        if (targetRpm <= 0.0) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            motorState = MotorState.OFF;
            return;
        }

        double ticksPerSec = rpmToTicksPerSec(targetRpm);
        leftMotor.setVelocity(ticksPerSec);
        rightMotor.setVelocity(ticksPerSec);

        // 3. Check: both motors must be inside tolerance to count as at speed.
        boolean leftOk = Math.abs(measuredLeftRpm - targetRpm) <= RobotConstants.Outtake.RPM_TOLERANCE;
        boolean rightOk = Math.abs(measuredRightRpm - targetRpm) <= RobotConstants.Outtake.RPM_TOLERANCE;
        motorState = (leftOk && rightOk) ? MotorState.AT_SPEED : MotorState.SPINNING_UP;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * RobotConstants.Outtake.TICKS_PER_REV / 60.0;
    }

    private static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / RobotConstants.Outtake.TICKS_PER_REV;
    }

    /**
     * @return the requested target RPM
     */
    public double getTargetRpm() {
        return targetRpm;
    }

    /**
     * @return left outtake motor RPM measured in the last {@link #update()}
     */
    public double getLeftRpm() {
        return measuredLeftRpm;
    }

    /**
     * @return right outtake motor RPM measured in the last {@link #update()}
     */
    public double getRightRpm() {
        return measuredRightRpm;
    }

    /**
     * @return average measured outtake RPM across both motors
     */
    public double getCurrentRpm() {
        return (measuredLeftRpm + measuredRightRpm) / 2.0;
    }

    /**
     * @return signed RPM error (target minus measured) of the left motor
     */
    public double getLeftRpmError() {
        return targetRpm - measuredLeftRpm;
    }

    /**
     * @return signed RPM error (target minus measured) of the right motor
     */
    public double getRightRpmError() {
        return targetRpm - measuredRightRpm;
    }

    /**
     * @return {@code true} if spinning and, as of the last {@link #update()},
     *         both motors were within {@link RobotConstants.Outtake#RPM_TOLERANCE}
     *         of {@link #getTargetRpm()}
     */
    public boolean atSpeed() {
        return motorState == MotorState.AT_SPEED;
    }

    /**
     * @return current motor state ({@code OFF}, {@code SPINNING_UP}, or {@code AT_SPEED})
     */
    public MotorState getMotorState() {
        return motorState;
    }
}
