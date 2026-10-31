package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Spins the intake and transfer motors independently so you can check wiring
 * and direction.
 *
 * <p>Hold right trigger to intake, left trigger to reverse intake, and hold A
 * to run the transfer motor.
 */
@TeleOp(name = "Test: Intake", group = RobotConstants.OpModeGroups.TEST)
public class IntakeTest extends OpMode {

    private IntakeSubsystem intake;

    /**
     * Connects to the intake and transfer motors.
     */
    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
        telemetry.addLine("RT = intake, LT = reverse intake, A = transfer");
        telemetry.update();
    }

    /**
     * Applies trigger and button commands and shows the resulting states.
     */
    @Override
    public void loop() {
        if (gamepad1.left_trigger > RobotConstants.Intake.TRIGGER_DEADZONE) {
            intake.reverse();
        } else if (gamepad1.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE) {
            intake.intake();
        } else {
            intake.stopIntake();
        }

        if (gamepad1.a) {
            intake.transfer();
        } else {
            intake.stopTransfer();
        }

        intake.update();
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Transfer State", intake.getTransferState());
        telemetry.update();
    }

    /**
     * Makes sure both motors stop spinning after the OpMode ends.
     */
    @Override
    public void stop() {
        if (intake != null) {
            intake.stop();
            intake.update();
        }
    }
}
