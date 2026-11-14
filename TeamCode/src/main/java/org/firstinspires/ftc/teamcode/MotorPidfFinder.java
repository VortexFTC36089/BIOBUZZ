package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Live velocity PIDF tuner for one motor.
 *
 * <p>Runs the selected motor at a target RPM using the hub's built-in velocity
 * PIDF ({@code RUN_USING_ENCODER} + {@code setVelocity()}), and lets you change
 * P, I, D and F from the gamepad while it spins. New gains are sent to the hub
 * the moment you change them. Nothing is saved: when the motor behaves, copy
 * the four numbers from telemetry into {@link RobotConstants}.
 *
 * <h3>Gamepad 1</h3>
 * <ul>
 *   <li>A: start / stop the motor</li>
 *   <li>Dpad Left / Right: pick which gain to edit (P, I, D, F)</li>
 *   <li>Dpad Up / Down: raise / lower that gain by the step</li>
 *   <li>B / X: step x10 / step ÷10</li>
 *   <li>Right / Left bumper: target RPM + / - {@link #RPM_STEP}</li>
 *   <li>Start / Back: next / previous motor in {@link #MOTOR_NAMES}</li>
 *   <li>Y: reset the selected motor's gains to its starting values</li>
 * </ul>
 *
 * <h3>Suggested order for a flywheel</h3>
 * Set P, I and D to 0. Raise F until the motor settles just under the target.
 * Raise P until it reaches the target and recovers quickly after a shot, and
 * back it off if the RPM starts to oscillate. Add a little D only if it
 * overshoots. Leave I at 0 unless it never quite reaches the target.
 */
@TeleOp(name = "Test: Motor PIDF Finder", group = RobotConstants.OpModeGroups.TEST)
public class MotorPidfFinder extends OpMode {

    // ---- Edit these for the motor you are tuning ----

    /** Motors that can be tuned. Only the selected one runs. */
    private static final String[] MOTOR_NAMES = {
            RobotConstants.Outtake.LEFT_MOTOR,
            RobotConstants.Outtake.RIGHT_MOTOR
    };

    /** Whether each motor in {@link #MOTOR_NAMES} is reversed (same order). */
    private static final boolean[] MOTOR_REVERSED = {false, true};

    /** Encoder ticks per output revolution, used to convert RPM to ticks/sec. */
    private static final double TICKS_PER_REV = RobotConstants.Outtake.TICKS_PER_REV;

    private static final double START_TARGET_RPM = RobotConstants.Outtake.TARGET_RPM;
    private static final double RPM_STEP = 100.0;

    /**
     * Gains loaded at init and restored by Y: one row per motor in
     * {@link #MOTOR_NAMES} (same order), each in P, I, D, F order.
     */
    private static final double[][] START_GAINS = {
            {
                    RobotConstants.Outtake.LEFT_KP,
                    RobotConstants.Outtake.LEFT_KI,
                    RobotConstants.Outtake.LEFT_KD,
                    RobotConstants.Outtake.LEFT_KF
            },
            {
                    RobotConstants.Outtake.RIGHT_KP,
                    RobotConstants.Outtake.RIGHT_KI,
                    RobotConstants.Outtake.RIGHT_KD,
                    RobotConstants.Outtake.RIGHT_KF
            }
    };

    // ---- End of settings ----

    private static final String[] GAIN_NAMES = {"P", "I", "D", "F"};
    private static final double MIN_STEP = 0.001;
    private static final double MAX_STEP = 100.0;

    private final DcMotorEx[] motors = new DcMotorEx[MOTOR_NAMES.length];
    /** Live gains, one row per motor; each motor keeps its own edits. */
    private final double[][] gains = new double[MOTOR_NAMES.length][];

    private int motorIndex = 0;
    private int gainIndex = 0;
    private double step = 1.0;
    private double targetRpm = START_TARGET_RPM;
    private boolean running = false;

    private final ToggleButton runToggle = new ToggleButton();
    private final ToggleButton gainLeft = new ToggleButton();
    private final ToggleButton gainRight = new ToggleButton();
    private final ToggleButton gainUp = new ToggleButton();
    private final ToggleButton gainDown = new ToggleButton();
    private final ToggleButton stepUp = new ToggleButton();
    private final ToggleButton stepDown = new ToggleButton();
    private final ToggleButton rpmUp = new ToggleButton();
    private final ToggleButton rpmDown = new ToggleButton();
    private final ToggleButton nextMotor = new ToggleButton();
    private final ToggleButton prevMotor = new ToggleButton();
    private final ToggleButton resetGains = new ToggleButton();

    /**
     * Finds every motor in {@link #MOTOR_NAMES} and applies the starting gains.
     */
    @Override
    public void init() {
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motor.setDirection(MOTOR_REVERSED[i]
                    ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setPower(0.0);
            motors[i] = motor;

            gains[i] = START_GAINS[i].clone();
            motor.setVelocityPIDFCoefficients(gains[i][0], gains[i][1], gains[i][2], gains[i][3]);
        }

        // Faster telemetry so RPM changes are visible while tuning.
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Motor PIDF Finder ready");
        telemetry.addLine("A = run/stop  |  Dpad L/R = pick gain  |  Dpad U/D = change gain");
        telemetry.addLine("B / X = step x10 / ÷10  |  RB / LB = target RPM +/-");
        telemetry.addLine("Start / Back = next / prev motor  |  Y = reset gains");
        telemetry.update();
    }

    /**
     * Reads the gamepad, pushes any changed gains to the hub, and commands the motor.
     */
    @Override
    public void loop() {
        // ---- Motor selection ----
        int motorChange = (nextMotor.update(gamepad1.start) ? 1 : 0)
                - (prevMotor.update(gamepad1.back) ? 1 : 0);
        if (motorChange != 0) {
            motors[motorIndex].setPower(0.0);
            motorIndex = (motorIndex + motorChange + motors.length) % motors.length;
        }
        double[] motorGains = gains[motorIndex];

        // ---- Gain selection and step size ----
        if (gainRight.update(gamepad1.dpad_right)) {
            gainIndex = (gainIndex + 1) % GAIN_NAMES.length;
        }
        if (gainLeft.update(gamepad1.dpad_left)) {
            gainIndex = (gainIndex - 1 + GAIN_NAMES.length) % GAIN_NAMES.length;
        }
        if (stepUp.update(gamepad1.b)) {
            step = Math.min(MAX_STEP, step * 10.0);
        }
        if (stepDown.update(gamepad1.x)) {
            step = Math.max(MIN_STEP, step / 10.0);
        }

        // ---- Gain edits (sent to the hub only when something changed) ----
        boolean gainsChanged = false;
        if (gainUp.update(gamepad1.dpad_up)) {
            motorGains[gainIndex] += step;
            gainsChanged = true;
        }
        if (gainDown.update(gamepad1.dpad_down)) {
            motorGains[gainIndex] = Math.max(0.0, motorGains[gainIndex] - step);
            gainsChanged = true;
        }
        if (resetGains.update(gamepad1.y)) {
            System.arraycopy(START_GAINS[motorIndex], 0, motorGains, 0, motorGains.length);
            gainsChanged = true;
        }
        if (gainsChanged) {
            applyGains();
        }

        // ---- Target RPM and run/stop ----
        if (rpmUp.update(gamepad1.right_bumper)) {
            targetRpm += RPM_STEP;
        }
        if (rpmDown.update(gamepad1.left_bumper)) {
            targetRpm = Math.max(0.0, targetRpm - RPM_STEP);
        }
        if (runToggle.update(gamepad1.a)) {
            running = !running;
        }

        // ---- Command the motor ----
        DcMotorEx motor = motors[motorIndex];
        if (running) {
            motor.setVelocity(targetRpm * TICKS_PER_REV / 60.0);
        } else {
            motor.setPower(0.0);
        }

        // ---- Telemetry ----
        double actualRpm = motor.getVelocity() * 60.0 / TICKS_PER_REV;
        telemetry.addData("Motor", "%s (%s)", MOTOR_NAMES[motorIndex], running ? "RUNNING" : "STOPPED");
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Actual RPM", "%.0f", actualRpm);
        telemetry.addData("Error RPM", "%.0f", running ? targetRpm - actualRpm : 0.0);
        telemetry.addData("Step", "%.3f", step);
        for (int i = 0; i < GAIN_NAMES.length; i++) {
            telemetry.addData((i == gainIndex ? "> " : "  ") + GAIN_NAMES[i], "%.3f", motorGains[i]);
        }
        telemetry.addLine("Copy P, I, D, F into RobotConstants when done.");
        telemetry.update();
    }

    /**
     * Stops every motor when the OpMode ends.
     */
    @Override
    public void stop() {
        for (DcMotorEx motor : motors) {
            if (motor != null) {
                motor.setPower(0.0);
            }
        }
    }

    /** Sends the current P, I, D, F to the selected motor's hub velocity controller. */
    private void applyGains() {
        double[] g = gains[motorIndex];
        motors[motorIndex].setVelocityPIDFCoefficients(g[0], g[1], g[2], g[3]);
    }
}
