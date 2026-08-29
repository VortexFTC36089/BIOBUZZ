package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Use this to check the intake by itself.
 *
 *   Right trigger  intake in
 *   Left trigger   intake reverse
 *   Release        stop
 *
 * If it spins the wrong way, change intake.setDirection in Robot.java.
 * If it is too strong or too weak, change INTAKE_SPEED or OUTTAKE_SPEED in Robot.java.
 *
 * The drive motors still need to be in the robot configuration, even in this test.
 */
@TeleOp(name = "Intake Test", group = "Testing")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("Right trigger = in. Left trigger = reverse.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                robot.intakeOn();
                telemetry.addData("intake", "IN");
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intakeReverse();
                telemetry.addData("intake", "REVERSE");
            } else {
                robot.intakeOff();
                telemetry.addData("intake", "OFF");
            }
            telemetry.update();
        }

        robot.stop();
    }
}
