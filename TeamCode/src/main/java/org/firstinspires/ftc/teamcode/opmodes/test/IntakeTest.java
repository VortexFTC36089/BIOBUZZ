package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Spins only the intake so you can check wiring and direction.
 *
 * <p>Hold right trigger to intake, left trigger to reverse. If "intake"
 * spits pieces out, flip {@code IntakeSubsystem} motor direction.
 */
@TeleOp(name = "Test: Intake", group = RobotConstants.OpModeGroups.TEST)
public class IntakeTest extends OpMode {

    private IntakeSubsystem intake;

    /**
     * Connects to the intake motor only.
     */
    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
        telemetry.addLine("RT = intake, LT = reverse");
        telemetry.update();
    }

    /**
     * Applies trigger commands and shows the resulting state.
     */
    @Override
    public void loop() {
        if (gamepad1.left_trigger > RobotConstants.Intake.TRIGGER_DEADZONE) {
            intake.reverse();
        } else if (gamepad1.right_trigger > RobotConstants.Intake.TRIGGER_DEADZONE) {
            intake.intake();
        } else {
            intake.stop();
        }

        intake.update();
        telemetry.addData("State", intake.getState());
        telemetry.update();
    }

    /**
     * Makes sure the roller cannot keep spinning after the OpMode ends.
     */
    @Override
    public void stop() {
        if (intake != null) {
            intake.stop();
            intake.update();
        }
    }
}
