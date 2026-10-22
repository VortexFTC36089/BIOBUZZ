package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testing", group = "testing")
public class testing extends LinearOpMode {

    // Configured device names matching your exact setup
    private final String[] MOTOR_NAMES = {"motor_0", "motor_1", "motor_2", "motor_3"};
    private final String[] SERVO_NAMES = {"servo_1", "servo_2", "servo_3"};

    private enum MotorMode { POWER, VELOCITY }
    private enum ServoType { POSITION, CONTINUOUS }

    @Override
    public void runOpMode() {
        // Hardware arrays
        DcMotorEx[] motors = new DcMotorEx[MOTOR_NAMES.length];
        Servo[] posServos = new Servo[SERVO_NAMES.length];
        CRServo[] crServos = new CRServo[SERVO_NAMES.length];

        // Initialize Motors
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            try {
                motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception e) {
                telemetry.addData("Warning", "Device missing: " + MOTOR_NAMES[i]);
            }
        }

        // Initialize Servos
        for (int i = 0; i < SERVO_NAMES.length; i++) {
            try {
                posServos[i] = hardwareMap.get(Servo.class, SERVO_NAMES[i]);
            } catch (Exception e) {}
            try {
                crServos[i] = hardwareMap.get(CRServo.class, SERVO_NAMES[i]);
            } catch (Exception e) {}
        }

        // State variables
        int currentMotorIndex = 0;
        MotorMode currentMotorMode = MotorMode.POWER;
        double motorPower = 0.0;
        double motorVelocity = 0.0; // in ticks/sec

        int currentServoIndex = 0;
        ServoType currentServoType = ServoType.POSITION;
        double servoPosition = 0.5;
        boolean crServoOn = false;

        // Edge detection state variables
        boolean prevDpadLeft1 = false, prevDpadRight1 = false;
        boolean prevDpadUp1 = false, prevDpadDown1 = false;
        boolean prevLeftBumper1 = false, prevRightBumper1 = false;

        boolean prevDpadLeft2 = false, prevDpadRight2 = false;
        boolean prevDpadUp2 = false, prevDpadDown2 = false;
        boolean prevLeftBumper2 = false, prevRightBumper2 = false;
        boolean prevA2 = false;

        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =======================================================
            // GAMEPAD 1: MOTOR CONTROL
            // =======================================================

            // Switch Selected Motor (Left/Right Bumpers)
            if (gamepad1.left_bumper && !prevLeftBumper1) {
                currentMotorIndex = (currentMotorIndex - 1 + MOTOR_NAMES.length) % MOTOR_NAMES.length;
            }
            if (gamepad1.right_bumper && !prevRightBumper1) {
                currentMotorIndex = (currentMotorIndex + 1) % MOTOR_NAMES.length;
            }

            // Switch Motor Control Mode (Dpad Left / Right)
            if (gamepad1.dpad_left && !prevDpadLeft1) {
                currentMotorMode = MotorMode.POWER;
            }
            if (gamepad1.dpad_right && !prevDpadRight1) {
                currentMotorMode = MotorMode.VELOCITY;
            }

            // Adjust Power/Velocity
            if (currentMotorMode == MotorMode.POWER) {
                motorPower = gamepad1.right_trigger - gamepad1.left_trigger;
            } else {
                if (gamepad1.dpad_up && !prevDpadUp1) {
                    motorVelocity += 100.0; // +100 ticks/sec
                }
                if (gamepad1.dpad_down && !prevDpadDown1) {
                    motorVelocity -= 100.0; // -100 ticks/sec
                }
            }

            // Apply command to motor
            DcMotorEx activeMotor = motors[currentMotorIndex];
            if (activeMotor != null) {
                if (currentMotorMode == MotorMode.POWER) {
                    activeMotor.setPower(motorPower);
                } else {
                    activeMotor.setVelocity(motorVelocity);
                }
            }

            // =======================================================
            // GAMEPAD 2: SERVO CONTROL
            // =======================================================

            // Switch Selected Servo (Left/Right Bumpers)
            if (gamepad2.left_bumper && !prevLeftBumper2) {
                currentServoIndex = (currentServoIndex - 1 + SERVO_NAMES.length) % SERVO_NAMES.length;
            }
            if (gamepad2.right_bumper && !prevRightBumper2) {
                currentServoIndex = (currentServoIndex + 1) % SERVO_NAMES.length;
            }

            // Switch Servo Type (Dpad Left = CR, Dpad Right = Position)
            if (gamepad2.dpad_left && !prevDpadLeft2) {
                currentServoType = ServoType.CONTINUOUS;
            }
            if (gamepad2.dpad_right && !prevDpadRight2) {
                currentServoType = ServoType.POSITION;
            }

            // Adjust Servo Output
            if (currentServoType == ServoType.POSITION) {
                if (gamepad2.dpad_up && !prevDpadUp2) {
                    servoPosition = Math.min(1.0, servoPosition + 0.05);
                }
                if (gamepad2.dpad_down && !prevDpadDown2) {
                    servoPosition = Math.max(0.0, servoPosition - 0.05);
                }

                Servo activePosServo = posServos[currentServoIndex];
                if (activePosServo != null) {
                    activePosServo.setPosition(servoPosition);
                }
            } else {
                if (gamepad2.a && !prevA2) {
                    crServoOn = !crServoOn;
                }

                CRServo activeCRServo = crServos[currentServoIndex];
                if (activeCRServo != null) {
                    activeCRServo.setPower(crServoOn ? 1.0 : 0.0);
                }
            }

            // Store previous button states for edge detection
            prevDpadLeft1 = gamepad1.dpad_left;
            prevDpadRight1 = gamepad1.dpad_right;
            prevDpadUp1 = gamepad1.dpad_up;
            prevDpadDown1 = gamepad1.dpad_down;
            prevLeftBumper1 = gamepad1.left_bumper;
            prevRightBumper1 = gamepad1.right_bumper;

            prevDpadLeft2 = gamepad2.dpad_left;
            prevDpadRight2 = gamepad2.dpad_right;
            prevDpadUp2 = gamepad2.dpad_up;
            prevDpadDown2 = gamepad2.dpad_down;
            prevLeftBumper2 = gamepad2.left_bumper;
            prevRightBumper2 = gamepad2.right_bumper;
            prevA2 = gamepad2.a;

            // =======================================================
            // TELEMETRY
            // =======================================================
            telemetry.addData("--- MOTOR (Gamepad 1) ---", "");
            telemetry.addData("Active", MOTOR_NAMES[currentMotorIndex]);
            telemetry.addData("Mode", currentMotorMode);
            if (currentMotorMode == MotorMode.POWER) {
                telemetry.addData("Power (Triggers)", "%.2f", motorPower);
            } else {
                telemetry.addData("Target Velocity (Dpad Up/Down)", "%.0f ticks/s", motorVelocity);
                if (activeMotor != null) {
                    telemetry.addData("Actual Velocity", "%.0f ticks/s", activeMotor.getVelocity());
                }
            }

            telemetry.addData("\n--- SERVO (Gamepad 2) ---", "");
            telemetry.addData("Active", SERVO_NAMES[currentServoIndex]);
            telemetry.addData("Type", currentServoType);
            if (currentServoType == ServoType.POSITION) {
                telemetry.addData("Position (Dpad Up/Down)", "%.2f", servoPosition);
            } else {
                telemetry.addData("CR Power (Button A)", crServoOn ? "1.0 (ON)" : "0.0 (OFF)");
            }

            telemetry.update();
        }
    }
}