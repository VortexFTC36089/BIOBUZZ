package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Spins one mecanum motor at a time without Pedro.
 *
 * <p>Use this on the stand to confirm HardwareMap names and which way each
 * wheel turns. Prefer the Pedro Mecanum AutoTuner on the floor for the
 * values that go into {@code Constants}.
 *
 * <p>A = front left, B = front right, X = back left, Y = back right.
 */
@TeleOp(name = "Test: Drive Directions", group = RobotConstants.OpModeGroups.TEST)
public class DrivetrainDirectionTest extends OpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    /**
     * Grabs the four drive motors by the names in {@link RobotConstants.Drive}.
     */
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.BACK_RIGHT);

        telemetry.addLine("A FL  B FR  X BL  Y BR  (0.3 power)");
        telemetry.update();
    }

    /**
     * Runs the held motor at a low power so a belt-up robot does not jump.
     */
    @Override
    public void loop() {
        double power = 0.30;
        frontLeft.setPower(gamepad1.a ? power : 0.0);
        frontRight.setPower(gamepad1.b ? power : 0.0);
        backLeft.setPower(gamepad1.x ? power : 0.0);
        backRight.setPower(gamepad1.y ? power : 0.0);

        telemetry.addData("FL", frontLeft.getPower());
        telemetry.addData("FR", frontRight.getPower());
        telemetry.addData("BL", backLeft.getPower());
        telemetry.addData("BR", backRight.getPower());
        telemetry.update();
    }

    /**
     * Zeros every drive motor.
     */
    @Override
    public void stop() {
        if (frontLeft != null) {
            frontLeft.setPower(0.0);
        }
        if (frontRight != null) {
            frontRight.setPower(0.0);
        }
        if (backLeft != null) {
            backLeft.setPower(0.0);
        }
        if (backRight != null) {
            backRight.setPower(0.0);
        }
    }
}
