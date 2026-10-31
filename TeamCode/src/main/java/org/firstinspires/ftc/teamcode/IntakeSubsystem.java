package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake and transfer subsystem with independent control of the intake and
 * transfer motors.
 *
 * <p>OpModes should call {@link #intake()}, {@link #reverse()}, or
 * {@link #stopIntake()} for the intake motor, and {@link #transfer()},
 * {@link #reverseTransfer()}, or {@link #stopTransfer()} for the transfer
 * motor, then call {@link #update()} once per loop so motor powers always
 * match the requested states. {@link #stop()} stops both motors and is always
 * safe to call from OpMode {@code stop()}.
 *
 * <p>Hardware:
 * <ul>
 *   <li>One intake {@link DcMotorEx} named {@link RobotConstants.Intake#MOTOR}</li>
 *   <li>One transfer {@link DcMotorEx} named {@link RobotConstants.Intake#TRANSFER_MOTOR}</li>
 * </ul>
 */
public class IntakeSubsystem {

    /**
     * What the intake motor is doing this loop.
     */
    public enum IntakeState {
        OFF,
        INTAKING,
        REVERSING
    }

    /**
     * What the transfer motor is doing this loop.
     */
    public enum TransferState {
        OFF,
        TRANSFERRING,
        REVERSING
    }

    private final DcMotorEx motor;
    private final DcMotorEx transferMotor;
    private IntakeState state;
    private TransferState transferState;

    /**
     * Finds the intake and transfer motors and puts both in a known-safe configuration.
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.MOTOR);
        transferMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.TRANSFER_MOTOR);

        // Brake so a game piece does not coast out when we command stop.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO(team): verify directions — FORWARD means positive power intakes/transfers
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run without encoder: we command percent power, not a target position.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        state = IntakeState.OFF;
        transferState = TransferState.OFF;
        motor.setPower(0.0);
        transferMotor.setPower(0.0);
    }

    // ---- Intake Motor Controls ----

    /**
     * Starts pulling game elements in with the intake motor at
     * {@link RobotConstants.Intake#INTAKE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void intake() {
        state = IntakeState.INTAKING;
    }

    /**
     * Starts pushing game elements out with the intake motor at
     * {@link RobotConstants.Intake#OUTTAKE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void reverse() {
        state = IntakeState.REVERSING;
    }

    /**
     * Stops only the intake motor while leaving the transfer motor state unchanged.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stopIntake() {
        state = IntakeState.OFF;
    }

    /**
     * Toggles the intake motor between {@link IntakeState#INTAKING} and {@link IntakeState#OFF}.
     */
    public void toggleIntake() {
        if (state == IntakeState.INTAKING) {
            stopIntake();
        } else {
            intake();
        }
    }

    // ---- Transfer Motor Controls ----

    /**
     * Runs the transfer motor forward at
     * {@link RobotConstants.Intake#TRANSFER_INTAKE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void transfer() {
        transferState = TransferState.TRANSFERRING;
    }

    /**
     * Runs the transfer motor in reverse at
     * {@link RobotConstants.Intake#TRANSFER_REVERSE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void reverseTransfer() {
        transferState = TransferState.REVERSING;
    }

    /**
     * Stops only the transfer motor while leaving the intake motor state unchanged.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stopTransfer() {
        transferState = TransferState.OFF;
    }

    /**
     * Requests zero power on both intake and transfer motors. Always safe to
     * call, including from OpMode {@code stop()}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stop() {
        state = IntakeState.OFF;
        transferState = TransferState.OFF;
    }

    /**
     * Writes motor powers for the current states to the intake and transfer
     * motors independently.
     *
     * <p>Call once per OpMode loop. Powers come from {@link RobotConstants}
     * so TeleOp never hardcodes a number.</p>
     */
    public void update() {
        switch (state) {
            case INTAKING:
                motor.setPower(RobotConstants.Intake.INTAKE_POWER);
                break;
            case REVERSING:
                motor.setPower(RobotConstants.Intake.OUTTAKE_POWER);
                break;
            case OFF:
            default:
                motor.setPower(0.0);
                break;
        }

        switch (transferState) {
            case TRANSFERRING:
                transferMotor.setPower(RobotConstants.Intake.TRANSFER_INTAKE_POWER);
                break;
            case REVERSING:
                transferMotor.setPower(RobotConstants.Intake.TRANSFER_REVERSE_POWER);
                break;
            case OFF:
            default:
                transferMotor.setPower(0.0);
                break;
        }
    }

    /**
     * @return current requested intake motor state ({@code OFF}, {@code INTAKING}, or {@code REVERSING})
     */
    public IntakeState getState() {
        return state;
    }

    /**
     * @return current requested transfer motor state ({@code OFF}, {@code TRANSFERRING}, or {@code REVERSING})
     */
    public TransferState getTransferState() {
        return transferState;
    }
}
