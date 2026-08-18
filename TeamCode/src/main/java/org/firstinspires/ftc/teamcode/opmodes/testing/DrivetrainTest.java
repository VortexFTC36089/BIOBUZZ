package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Isolated drivetrain hardware check.
 *
 * Uses the same motor directions as competition code so encoder signs match
 * production. Run this before trusting Main TeleOp.
 *
 *   D-pad Up     leftFront
 *   D-pad Down   leftRear
 *   D-pad Right  rightFront
 *   D-pad Left   rightRear
 *   A            all motors forward
 *   B            all motors reverse
 *   Y            reset encoders
 *
 * After A is held, the robot should drive straight forward and all encoder
 * counts should increase.
 */
@TeleOp(name = "Drivetrain Test", group = "Testing")
public class DrivetrainTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        hw.leftFront.setPower(0);
        hw.rightFront.setPower(0);
        hw.leftRear.setPower(0);
        hw.rightRear.setPower(0);
        hw.intake.setPower(0);

        telemetry.addLine("Drivetrain Test ready. Robot will move at test power.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double lf = 0;
            double rf = 0;
            double lr = 0;
            double rr = 0;
            String active = "none";
            double power = Constants.Drive.TEST_POWER;

            if (gamepad1.dpad_up) {
                lf = power;
                active = Constants.Drive.LEFT_FRONT;
            } else if (gamepad1.dpad_down) {
                lr = power;
                active = Constants.Drive.LEFT_REAR;
            } else if (gamepad1.dpad_right) {
                rf = power;
                active = Constants.Drive.RIGHT_FRONT;
            } else if (gamepad1.dpad_left) {
                rr = power;
                active = Constants.Drive.RIGHT_REAR;
            } else if (gamepad1.a) {
                lf = rf = lr = rr = power;
                active = "ALL forward";
            } else if (gamepad1.b) {
                lf = rf = lr = rr = -power;
                active = "ALL reverse";
            }

            if (gamepad1.y) {
                hw.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hw.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hw.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hw.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            hw.leftFront.setPower(lf);
            hw.rightFront.setPower(rf);
            hw.leftRear.setPower(lr);
            hw.rightRear.setPower(rr);

            telemetry.addData("Active", active);
            telemetry.addData("leftFront  pwr/enc", "%.2f / %d", lf, hw.leftFront.getCurrentPosition());
            telemetry.addData("rightFront pwr/enc", "%.2f / %d", rf, hw.rightFront.getCurrentPosition());
            telemetry.addData("leftRear   pwr/enc", "%.2f / %d", lr, hw.leftRear.getCurrentPosition());
            telemetry.addData("rightRear  pwr/enc", "%.2f / %d", rr, hw.rightRear.getCurrentPosition());
            telemetry.addLine("DPad: one motor | A: all fwd | B: all rev | Y: reset enc");
            telemetry.update();
        }

        hw.leftFront.setPower(0);
        hw.rightFront.setPower(0);
        hw.leftRear.setPower(0);
        hw.rightRear.setPower(0);
    }
}
