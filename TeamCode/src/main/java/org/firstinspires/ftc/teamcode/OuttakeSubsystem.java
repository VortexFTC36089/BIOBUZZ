package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Outtake subsystem consisting of an outtake motor and a two-position servo gate.
 *
 * <p>OpModes should call {@link #outtake()}, {@link #reverse()}, or
 * {@link #stopMotor()} for the motor, and {@link #openGate()},
 * {@link #closeGate()}, or {@link #toggleGate()} for the gate servo, then call
 * {@link #update()} once per loop so hardware outputs always match the
 * requested state.
 *
 * <p>Hardware:
 * <ul>
 *   <li>One {@link DcMotorEx} named {@link RobotConstants.Outtake#MOTOR}</li>
 *   <li>One {@link Servo} named {@link RobotConstants.Outtake#GATE_SERVO}</li>
 * </ul>
 */
public class OuttakeSubsystem {

    /**
     * What the outtake motor is doing this loop.
     */
    public enum MotorState {
        OFF,
        OUTTAKING,
        REVERSING
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

    /**
     * Finds the outtake motor and gate servo, and puts both in a known-safe
     * initial state (motor stopped, gate closed).
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     */
    public OuttakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.MOTOR);
        gateServo = hardwareMap.get(Servo.class, RobotConstants.Outtake.GATE_SERVO);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO(team): verify direction — FORWARD means positive power scores/ejects
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorState = MotorState.OFF;
        gateState = GateState.CLOSED;

        motor.setPower(0.0);
        gateServo.setPosition(RobotConstants.Outtake.GATE_CLOSED_POSITION);
    }

    /**
     * Runs the outtake motor forward at {@link RobotConstants.Outtake#OUTTAKE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void outtake() {
        motorState = MotorState.OUTTAKING;
    }

    /**
     * Runs the outtake motor in reverse at {@link RobotConstants.Outtake#REVERSE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void reverse() {
        motorState = MotorState.REVERSING;
    }

    /**
     * Requests zero power on the outtake motor while leaving the gate state unchanged.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stopMotor() {
        motorState = MotorState.OFF;
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
        motorState = MotorState.OFF;
        gateState = GateState.CLOSED;
    }

    /**
     * Writes motor power and servo gate position for the current states.
     *
     * <p>Call once per OpMode loop.</p>
     */
    public void update() {
        switch (motorState) {
            case OUTTAKING:
                motor.setPower(RobotConstants.Outtake.OUTTAKE_POWER);
                break;
            case REVERSING:
                motor.setPower(RobotConstants.Outtake.REVERSE_POWER);
                break;
            case OFF:
            default:
                motor.setPower(0.0);
                break;
        }

        if (gateState == GateState.OPEN) {
            gateServo.setPosition(RobotConstants.Outtake.GATE_OPEN_POSITION);
        } else {
            gateServo.setPosition(RobotConstants.Outtake.GATE_CLOSED_POSITION);
        }
    }

    /**
     * @return current requested motor state ({@code OFF}, {@code OUTTAKING}, or {@code REVERSING})
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
