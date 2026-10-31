package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Outtake subsystem consisting of a velocity-controlled outtake motor (RPM)
 * and a two-position servo gate.
 *
 * <p>OpModes should call {@link #outtake()} (or {@link #setTargetRpm(double)})
 * or {@link #stopMotor()} for the motor, and {@link #openGate()},
 * {@link #closeGate()}, or {@link #toggleGate()} for the gate servo, then call
 * {@link #update()} once per loop so hardware outputs always match the
 * requested state. The outtake motor is one-way only and cannot reverse.
 *
 * <p>Hardware:
 * <ul>
 *   <li>One {@link DcMotorEx} with an encoder named {@link RobotConstants.Outtake#MOTOR}</li>
 *   <li>One {@link Servo} named {@link RobotConstants.Outtake#GATE_SERVO}</li>
 * </ul>
 */
public class OuttakeSubsystem {

    /**
     * What the outtake motor is doing this loop.
     */
    public enum MotorState {
        OFF,
        OUTTAKING
    }

    /**
     * Whether the servo gate is holding or releasing game pieces.
     */
    public enum GateState {
        CLOSED,
        OPEN
    }

    private final DcMotorEx motor;
    private final Servo gateServo;

    private MotorState motorState;
    private GateState gateState;
    private double targetRpm;

    /**
     * Finds the outtake motor and gate servo, and puts both in a known-safe
     * initial state (motor stopped in velocity mode, gate closed).
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     */
    public OuttakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.MOTOR);
        gateServo = hardwareMap.get(Servo.class, RobotConstants.Outtake.GATE_SERVO);

        // Float so the flywheel spins down smoothly instead of slamming to a stop.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // TODO(team): verify direction — FORWARD means positive RPM scores/ejects
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // RUN_USING_ENCODER lets setVelocity() hold target RPM with the hub's PIDF.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorState = MotorState.OFF;
        gateState = GateState.CLOSED;
        targetRpm = 0.0;

        motor.setPower(0.0);
        gateServo.setPosition(RobotConstants.Outtake.GATE_CLOSED_POSITION);
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
     * Requests zero RPM on the outtake motor while leaving the gate state unchanged.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stopMotor() {
        setTargetRpm(0.0);
    }

    /**
     * Opens the servo gate to release game pieces.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void openGate() {
        gateState = GateState.OPEN;
    }

    /**
     * Closes the servo gate to hold game pieces inside the outtake.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void closeGate() {
        gateState = GateState.CLOSED;
    }

    /**
     * Toggles the servo gate between {@link GateState#OPEN} and {@link GateState#CLOSED}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void toggleGate() {
        if (gateState == GateState.CLOSED) {
            openGate();
        } else {
            closeGate();
        }
    }

    /**
     * Stops the outtake motor and closes the gate. Always safe to call from
     * OpMode {@code stop()}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stop() {
        stopMotor();
        gateState = GateState.CLOSED;
    }

    /**
     * Writes motor velocity (ticks/sec) and servo gate position for the current states.
     *
     * <p>Call once per OpMode loop.</p>
     */
    public void update() {
        if (motorState == MotorState.OFF || targetRpm <= 0.0) {
            motor.setPower(0.0);
        } else {
            motor.setVelocity(targetRpm * RobotConstants.Outtake.TICKS_PER_REV / 60.0);
        }

        if (gateState == GateState.OPEN) {
            gateServo.setPosition(RobotConstants.Outtake.GATE_OPEN_POSITION);
        } else {
            gateServo.setPosition(RobotConstants.Outtake.GATE_CLOSED_POSITION);
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

    /**
     * @return current requested gate state ({@code CLOSED} or {@code OPEN})
     */
    public GateState getGateState() {
        return gateState;
    }

    /**
     * @return {@code true} if the gate is currently commanded open
     */
    public boolean isGateOpen() {
        return gateState == GateState.OPEN;
    }
}
