package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This file is the robot.
 *
 * Open this file if:
 *   - a motor name is wrong (robot crashes on INIT)
 *   - a wheel spins the wrong way
 *   - intake is too fast or too slow
 *
 * How to add a new motor:
 *   1. Add it on the Driver Hub and give it a name
 *   2. Add a "public DcMotor ..." line below
 *   3. Get it from hardwareMap in the Robot() method
 *   4. Write a small method that sets its power
 *
 * Do not edit files inside FtcRobotController. Team code lives here only.
 */
public class Robot {

    // ===== NAMES (must match the Driver Hub configuration EXACTLY) =====
    public static final String LEFT_FRONT_NAME  = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_REAR_NAME   = "leftRear";
    public static final String RIGHT_REAR_NAME  = "rightRear";
    public static final String INTAKE_NAME      = "intake";

    // ===== SPEEDS you can change (0.0 = stop, 1.0 = full power) =====
    public static final double INTAKE_SPEED  = 0.8;   // pulling game pieces in
    public static final double OUTTAKE_SPEED = 0.6;   // pushing game pieces out
    public static final double SLOW_MODE     = 0.35;  // used when the driver holds the bumper

    // The motors. Other files use these, for example: robot.leftFront.setPower(0.3);
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    public DcMotor intake;

    public Robot(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotor.class, LEFT_FRONT_NAME);
        rightFront = hardwareMap.get(DcMotor.class, RIGHT_FRONT_NAME);
        leftRear   = hardwareMap.get(DcMotor.class, LEFT_REAR_NAME);
        rightRear  = hardwareMap.get(DcMotor.class, RIGHT_REAR_NAME);
        intake     = hardwareMap.get(DcMotor.class, INTAKE_NAME);

        // BRAKE = robot holds still when you let go of the sticks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // If a wheel runs backward when it should go forward, swap FORWARD and REVERSE
        // on THAT motor only. Test with DriveTest first.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Drive the mecanum wheels.
     *
     * forward:  1.0 = forward,  -1.0 = backward
     * strafe:   1.0 = right,    -1.0 = left
     * turn:     1.0 = turn left, -1.0 = turn right
     *
     * You usually do not need to change the four math lines.
     */
    public void drive(double forward, double strafe, double turn) {
        double leftFrontPower  = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftRearPower   = forward - strafe + turn;
        double rightRearPower  = forward + strafe - turn;

        // Motors cannot go faster than 1.0. If the mix is too big, scale all four
        // the same amount so the robot still goes the direction we asked for.
        double biggest = Math.abs(leftFrontPower);
        if (Math.abs(rightFrontPower) > biggest) biggest = Math.abs(rightFrontPower);
        if (Math.abs(leftRearPower) > biggest) biggest = Math.abs(leftRearPower);
        if (Math.abs(rightRearPower) > biggest) biggest = Math.abs(rightRearPower);

        if (biggest > 1.0) {
            leftFrontPower  = leftFrontPower / biggest;
            rightFrontPower = rightFrontPower / biggest;
            leftRearPower   = leftRearPower / biggest;
            rightRearPower  = rightRearPower / biggest;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    public void intakeOn() {
        intake.setPower(INTAKE_SPEED);
    }

    public void intakeReverse() {
        intake.setPower(-OUTTAKE_SPEED);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    // Stop everything. Call this at the end of a program.
    public void stop() {
        drive(0, 0, 0);
        intakeOff();
    }
}
