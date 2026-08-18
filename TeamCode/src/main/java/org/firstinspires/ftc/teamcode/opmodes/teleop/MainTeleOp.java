package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Competition TeleOp.
 *
 * DRIVER (gamepad1)
 *   Left stick Y     Forward / backward
 *   Left stick X     Strafe left / right
 *   Right stick X    Rotate
 *   Right bumper     Slow mode (hold)
 *   Right trigger    Intake (single-controller fallback)
 *   Left trigger     Outtake (single-controller fallback)
 *
 * OPERATOR (gamepad2)
 *   Right trigger    Intake (hold)
 *   Left trigger     Outtake (hold)
 *
 * Stick Y is negated because FTC gamepads report up as negative.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("Initialized. Confirm hardware names match Driver Hub config.");
        telemetry.addData("Drive motors", "%s %s %s %s",
                Constants.Drive.LEFT_FRONT,
                Constants.Drive.RIGHT_FRONT,
                Constants.Drive.LEFT_REAR,
                Constants.Drive.RIGHT_REAR);
        telemetry.addData("Intake motor", Constants.Intake.MOTOR);
        telemetry.update();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            loopTimer.reset();

            double axial = applyDeadzone(-gamepad1.left_stick_y);
            double lateral = applyDeadzone(gamepad1.left_stick_x);
            double yaw = applyDeadzone(gamepad1.right_stick_x);

            robot.drivetrain.setSlowMode(gamepad1.right_bumper);
            robot.drivetrain.drive(axial, lateral, yaw);

            double intakeTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            double outtakeTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);

            if (intakeTrigger > Constants.Intake.TRIGGER_THRESHOLD) {
                robot.intake.intake();
            } else if (outtakeTrigger > Constants.Intake.TRIGGER_THRESHOLD) {
                robot.intake.outtake();
            } else {
                robot.intake.stop();
            }

            telemetry.addData("Drive", "axial=%.2f  lateral=%.2f  yaw=%.2f  slow=%b",
                    axial, lateral, yaw, robot.drivetrain.isSlowMode());
            telemetry.addData("Intake", robot.intake.getState());
            telemetry.addData("Loop ms", "%.1f", loopTimer.milliseconds());
            telemetry.update();
        }

        robot.stopAll();
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < Constants.Drive.STICK_DEADZONE ? 0.0 : value;
    }
}
