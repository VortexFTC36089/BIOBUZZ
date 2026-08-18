package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.Constants;

/**
 * Single point of hardware initialization for the robot.
 *
 * OpModes and subsystems should not call {@code hardwareMap.get()} themselves.
 * Names come from {@link Constants} and must match the Driver Hub configuration.
 *
 * This constructor requires every configured device to exist. Missing devices
 * fail during INIT so wiring/config mistakes are visible before the match starts.
 */
public final class RobotHardware {

    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx leftRear;
    public final DcMotorEx rightRear;
    public final DcMotorEx intake;

    public RobotHardware(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotorEx.class, Constants.Drive.LEFT_FRONT);
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.Drive.RIGHT_FRONT);
        leftRear   = hardwareMap.get(DcMotorEx.class, Constants.Drive.LEFT_REAR);
        rightRear  = hardwareMap.get(DcMotorEx.class, Constants.Drive.RIGHT_REAR);
        intake     = hardwareMap.get(DcMotorEx.class, Constants.Intake.MOTOR);

        configureDriveMotors();
        configureIntakeMotor();
    }

    private void configureDriveMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Standard mecanum: left side reversed so positive power is forward.
        // If the robot drives the wrong way, reverse individual motors here after
        // running Drivetrain Test — do not scatter direction changes in OpModes.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void configureIntakeMotor() {
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
