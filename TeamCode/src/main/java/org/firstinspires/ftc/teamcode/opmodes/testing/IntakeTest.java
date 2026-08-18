package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Isolated intake check.
 *
 * Right trigger — intake
 * Left trigger  — outtake
 * Release       — stop
 *
 * This OpMode still constructs full {@link RobotHardware}, so the four drive
 * motors must also exist in the robot configuration.
 */
@TeleOp(name = "Intake Test", group = "Testing")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        Intake intake = new Intake(hw);

        telemetry.addData("Intake motor", Constants.Intake.MOTOR);
        telemetry.addLine("Right trigger = intake | Left trigger = outtake");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > Constants.Intake.TRIGGER_THRESHOLD) {
                intake.intake();
            } else if (gamepad1.left_trigger > Constants.Intake.TRIGGER_THRESHOLD) {
                intake.outtake();
            } else {
                intake.stop();
            }

            telemetry.addData("State", intake.getState());
            telemetry.addData("Right trigger", "%.2f", gamepad1.right_trigger);
            telemetry.addData("Left trigger", "%.2f", gamepad1.left_trigger);
            telemetry.update();
        }

        intake.stop();
    }
}
