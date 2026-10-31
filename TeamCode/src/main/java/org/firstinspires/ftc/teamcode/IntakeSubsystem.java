package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake and transfer subsystem.
 *
 * <p>OpModes should call {@link #intake()}, {@link #reverse()},
 * {@link #stop()}, or {@link #toggleIntake()} from gamepad handlers, then
 * call {@link #update()} once per loop so the intake and transfer motor powers
 * always match {@link IntakeState}. That way {@link #stop()} is always safe:
 * it only changes state, and {@code update()} is what actually writes power.
 *
 * <p>Hardware:
 * <ul>
 *   <li>One intake {@link DcMotorEx} named {@link RobotConstants.Intake#MOTOR}</li>
 *   <li>One transfer {@link DcMotorEx} named {@link RobotConstants.Intake#TRANSFER_MOTOR}</li>
 * </ul>
 */
public class IntakeSubsystem {

    /**
     * What the intake and transfer are trying to do this loop.
     */
    public enum IntakeState {
        OFF,
        INTAKING,
        REVERSING
    }

    private final DcMotorEx motor;
    private final DcMotorEx transferMotor;
    private IntakeState state;

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
        motor.setPower(0.0);
        transferMotor.setPower(0.0);
    }

    /**
     * Starts pulling game elements in with both the intake and transfer motors.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void intake() {
        state = IntakeState.INTAKING;
    }

    /**
     * Starts pushing game elements out with both the intake and transfer motors.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void reverse() {
        state = IntakeState.REVERSING;
    }

    /**
     * Requests zero power on both intake and transfer motors. Always safe to
     * call, including from {@code stop()}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void stop() {
        state = IntakeState.OFF;
    }

    /**
     * Toggles between {@link IntakeState#INTAKING} and {@link IntakeState#OFF}.
     *
     * <p>If the intake is reversing, a toggle goes to OFF so one press always
     * means "stop whatever is happening" instead of jumping to intake.
     */
    public void toggleIntake() {
        if (state == IntakeState.INTAKING) {
            stop();
        } else {
            intake();
        }
    }

    /**
     * Writes motor powers for the current state to both the intake and transfer
     * motors.
     *
     * <p>Call once per OpMode loop. Powers come from {@link RobotConstants}
     * so TeleOp never hardcodes a number.</p>
     */
    public void update() {
        switch (state) {
            case INTAKING:
                motor.setPower(RobotConstants.Intake.INTAKE_POWER);
                transferMotor.setPower(RobotConstants.Intake.TRANSFER_INTAKE_POWER);
                break;
            case REVERSING:
                motor.setPower(RobotConstants.Intake.OUTTAKE_POWER);
                transferMotor.setPower(RobotConstants.Intake.TRANSFER_REVERSE_POWER);
                break;
            case OFF:
            default:
                motor.setPower(0.0);
                transferMotor.setPower(0.0);
                break;
        }
    }

    /**
     * Current requested state. Useful for telemetry.
     *
     * @return OFF, INTAKING, or REVERSING
     */
    public IntakeState getState() {
        return state;
    }
}
