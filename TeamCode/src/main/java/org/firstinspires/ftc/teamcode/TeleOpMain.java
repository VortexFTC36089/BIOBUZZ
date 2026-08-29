package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is the program you run during the driver-controlled period.
 *
 * Controller (gamepad 1):
 *   Left stick up/down     drive forward / backward
 *   Left stick left/right  strafe left / right
 *   Right stick left/right turn
 *   Right bumper (hold)    slow mode
 *   Right trigger          intake in
 *   Left trigger           intake reverse
 *
 * Gamepad 2 can also use the triggers for intake.
 *
 * To change a button, edit this file. To change a motor speed or direction,
 * edit Robot.java instead.
 */
@TeleOp(name = "TeleOp", group = "Competition")
public class TeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("Ready. Press START on the Driver Station.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // FTC controllers report "stick up" as a negative number, so we flip it.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Ignore tiny stick values when the stick is at rest (stops motor buzzing).
            if (Math.abs(forward) < 0.05) {
                forward = 0;
            }
            if (Math.abs(strafe) < 0.05) {
                strafe = 0;
            }
            if (Math.abs(turn) < 0.05) {
                turn = 0;
            }

            // Hold right bumper to drive slowly (helpful for lining up).
            if (gamepad1.right_bumper) {
                forward = forward * Robot.SLOW_MODE;
                strafe = strafe * Robot.SLOW_MODE;
                turn = turn * Robot.SLOW_MODE;
            }

            robot.drive(forward, strafe, turn);

            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                robot.intakeOn();
            } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                robot.intakeReverse();
            } else {
                robot.intakeOff();
            }

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            telemetry.update();
        }

        robot.stop();
    }
}
