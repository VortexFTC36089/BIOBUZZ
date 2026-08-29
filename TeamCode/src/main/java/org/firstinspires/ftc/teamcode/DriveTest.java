package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Use this to check that each wheel is plugged in correctly.
 *
 *   D-pad Up     leftFront
 *   D-pad Down   leftRear
 *   D-pad Right  rightFront
 *   D-pad Left   rightRear
 *   A            all wheels forward
 *   B            all wheels backward
 *   Y            reset encoder counts to 0
 *
 * What you should see:
 *   Hold A. The robot should drive straight forward.
 *   Encoder numbers should go up on every motor.
 *
 * If one wheel goes the wrong way, change that motor's direction in Robot.java.
 */
@TeleOp(name = "Drive Test", group = "Testing")
public class DriveTest extends LinearOpMode {

    // Slow on purpose so you can watch each wheel.
    public static final double TEST_SPEED = 0.3;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.stop();

        telemetry.addLine("Drive Test ready. Robot will move slowly.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double leftFrontPower = 0;
            double rightFrontPower = 0;
            double leftRearPower = 0;
            double rightRearPower = 0;
            String which = "none";

            if (gamepad1.dpad_up) {
                leftFrontPower = TEST_SPEED;
                which = "leftFront";
            } else if (gamepad1.dpad_down) {
                leftRearPower = TEST_SPEED;
                which = "leftRear";
            } else if (gamepad1.dpad_right) {
                rightFrontPower = TEST_SPEED;
                which = "rightFront";
            } else if (gamepad1.dpad_left) {
                rightRearPower = TEST_SPEED;
                which = "rightRear";
            } else if (gamepad1.a) {
                leftFrontPower = TEST_SPEED;
                rightFrontPower = TEST_SPEED;
                leftRearPower = TEST_SPEED;
                rightRearPower = TEST_SPEED;
                which = "all forward";
            } else if (gamepad1.b) {
                leftFrontPower = -TEST_SPEED;
                rightFrontPower = -TEST_SPEED;
                leftRearPower = -TEST_SPEED;
                rightRearPower = -TEST_SPEED;
                which = "all backward";
            }

            if (gamepad1.y) {
                robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            robot.leftFront.setPower(leftFrontPower);
            robot.rightFront.setPower(rightFrontPower);
            robot.leftRear.setPower(leftRearPower);
            robot.rightRear.setPower(rightRearPower);

            telemetry.addData("running", which);
            telemetry.addData("leftFront encoder", robot.leftFront.getCurrentPosition());
            telemetry.addData("rightFront encoder", robot.rightFront.getCurrentPosition());
            telemetry.addData("leftRear encoder", robot.leftRear.getCurrentPosition());
            telemetry.addData("rightRear encoder", robot.rightRear.getCurrentPosition());
            telemetry.update();
        }

        robot.stop();
    }
}
