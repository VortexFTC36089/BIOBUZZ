package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Mecanum drivetrain.
 *
 * Coordinate convention (same as FTC sample BasicOmniOpMode):
 *   axial   — forward / backward (positive = forward)
 *   lateral — strafe (positive = right)
 *   yaw     — rotation (positive = counterclockwise / turn left)
 *
 * Hardware: four DcMotorEx motors in an X-pattern mecanum layout.
 */
public class Drivetrain {

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    private boolean slowMode;

    public Drivetrain(RobotHardware hw) {
        leftFront  = hw.leftFront;
        rightFront = hw.rightFront;
        leftRear   = hw.leftRear;
        rightRear  = hw.rightRear;
    }

    /**
     * Drive using mecanum kinematics. Inputs are expected in the range [-1, 1].
     */
    public void drive(double axial, double lateral, double yaw) {
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftRearPower   = axial - lateral + yaw;
        double rightRearPower  = axial + lateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftRearPower   /= max;
            rightRearPower  /= max;
        }

        double scale = slowMode ? Constants.Drive.SLOW_MODE_FACTOR : Constants.Drive.MAX_POWER;

        leftFront.setPower(leftFrontPower * scale);
        rightFront.setPower(rightFrontPower * scale);
        leftRear.setPower(leftRearPower * scale);
        rightRear.setPower(rightRearPower * scale);
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void setSlowMode(boolean enabled) {
        slowMode = enabled;
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public int getLeftFrontPosition() {
        return leftFront.getCurrentPosition();
    }

    public int getRightFrontPosition() {
        return rightFront.getCurrentPosition();
    }

    public int getLeftRearPosition() {
        return leftRear.getCurrentPosition();
    }

    public int getRightRearPosition() {
        return rightRear.getCurrentPosition();
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
