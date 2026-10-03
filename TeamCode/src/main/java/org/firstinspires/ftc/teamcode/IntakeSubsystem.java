package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Single-motor intake.
 *
 * <p>OpModes should call {@link #intake()}, {@link #reverse()},
 * {@link #stop()}, or {@link #toggleIntake()} from gamepad handlers, then
 * call {@link #update()} once per loop so the motor power always matches
 * {@link IntakeState}. That way {@link #stop()} is always safe: it only
 * changes state, and {@code update()} is what actually writes power.
 *
 * <p>Hardware: one {@link DcMotorEx} named {@link RobotConstants.Intake#MOTOR}.
 */
public class IntakeSubsystem {

    /**
     * What the intake is trying to do this loop.
     */
    public enum IntakeState {
        OFF,
        INTAKING,
        REVERSING
    }

    private final DcMotorEx motor;
    private IntakeState state;

    /**
     * Finds the intake motor and puts it in a known-safe configuration.
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.MOTOR);

        // Brake so a game piece does not coast out when we command stop.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO(team): verify direction — FORWARD means positive power intakes
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run without encoder: we command percent power, not a target position.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        state = IntakeState.OFF;
        motor.setPower(0.0);
    }

    /**
     * Starts pulling game elements in at {@link RobotConstants.Intake#INTAKE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void intake() {
        state = IntakeState.INTAKING;
    }

    /**
     * Starts pushing game elements out at {@link RobotConstants.Intake#OUTTAKE_POWER}.
     *
     * <p>Takes effect on the next {@link #update()}.</p>
     */
    public void reverse() {
        state = IntakeState.REVERSING;
    }

    /**
     * Requests zero power. Always safe to call, including from {@code stop()}.
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
     * Writes motor power for the current state.
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
