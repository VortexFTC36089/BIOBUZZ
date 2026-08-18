package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Roller / belt intake.
 *
 * States:
 *   INTAKING  — collect game elements
 *   OUTTAKING — expel game elements
 *   STOPPED   — motor off
 */
public class Intake {

    public enum State {
        INTAKING,
        OUTTAKING,
        STOPPED
    }

    private final DcMotorEx motor;
    private State currentState = State.STOPPED;

    public Intake(RobotHardware hw) {
        motor = hw.intake;
    }

    public void intake() {
        motor.setPower(Constants.Intake.INTAKE_POWER);
        currentState = State.INTAKING;
    }

    public void outtake() {
        motor.setPower(Constants.Intake.OUTTAKE_POWER);
        currentState = State.OUTTAKING;
    }

    public void stop() {
        motor.setPower(0);
        currentState = State.STOPPED;
    }

    public State getState() {
        return currentState;
    }
}
