package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Single place for hardware map names and non-Pedro tunable values.
 *
 * <p>OpModes and subsystems should read names and powers from here instead of
 * scattering magic strings and numbers. Pedro Pathing still owns its own
 * drivetrain/localizer/algorithm constants; those live in
 * {@code pedroPathing.Constants} once the localizer is chosen.
 *
 * <p>Hardware this file currently assumes:
 * <ul>
 *   <li>Four mecanum drive motors: {@code frontLeft}, {@code frontRight},
 *       {@code backLeft}, {@code backRight}</li>
 *   <li>One intake {@link com.qualcomm.robotcore.hardware.DcMotor}: {@code intake}</li>
 *   <li>One transfer {@link com.qualcomm.robotcore.hardware.DcMotor}: {@code transfer}</li>
 *   <li>One outtake {@link com.qualcomm.robotcore.hardware.DcMotor}: {@code outtake}</li>
 *   <li>One outtake gate {@link com.qualcomm.robotcore.hardware.Servo}: {@code servo}</li>
 *   <li>goBILDA Pinpoint localizer named {@code pinpoint}</li>
 * </ul>
 */
public final class RobotConstants {

    /**
     * Prevents anyone from accidentally constructing this holder class.
     */
    private RobotConstants() {
        // Utility class: constants only.
    }

    /**
     * Driver Station groups used on {@code @TeleOp} / {@code @Autonomous}
     * annotations so the OpMode list stays organized.
     */
    public static final class OpModeGroups {
        public static final String TELEOP = "TeleOp";
        public static final String AUTO = "Auto";
        public static final String TEST = "Test";
        public static final String SAMPLES = "Samples";
        public static final String PEDRO = "Pedro";
    }

    /**
     * Hardware map names and TeleOp drive helpers for the mecanum chassis.
     *
     * <p>These strings must match the Robot Controller configuration exactly.
     * Motor directions are intentionally not stored here; Pedro's drivetrain
     * config owns direction because the follower is what commands the wheels.
     */
    public static final class Drive {
        public static final String FRONT_LEFT = "left_front";
        public static final String FRONT_RIGHT = "right_front";
        public static final String BACK_LEFT = "left_back";
        public static final String BACK_RIGHT = "right_back";

        /**
         * Scale applied to stick input while slow mode is held.
         *
         * <p>0.3 means 30% of normal speed, which is slow enough for scoring
         * alignment without making the robot feel stalled.
         */
        // TODO(team): tune after first driver practice
        public static final double SLOW_MODE_MULTIPLIER = 0.30;

        /**
         * Stick magnitude that counts as "the driver is moving" when we cancel
         * an automated drive-to-pose.
         *
         * <p>A small deadzone avoids cancelling because of stick drift.
         */
        public static final double AUTOMATION_CANCEL_DEADZONE = 0.15;
    }

    /**
     * goBILDA Pinpoint odometry computer.
     *
     * <p>Pedro docs: plug the Pinpoint into an I2C port other than 0 (port 0
     * is the Control Hub IMU), sticker/ports facing up, forward pod in the
     * Pinpoint X port and strafe pod in the Y port.
     */
    public static final class Localizer {
        public static final String PINPOINT = "pinpoint";
    }

    /**
     * Hardware map names and power values for the intake and transfer motors.
     *
     * <p>Powers are unitless motor commands in {@code [-1.0, 1.0]}. Positive
     * vs negative is a hardware convention; we treat positive as "intake"
     * until the team verifies the motor wiring.
     */
    public static final class Intake {
        public static final String MOTOR = "intake";
        public static final String TRANSFER_MOTOR = "transfer";

        /**
         * Power used on the intake motor while pulling game elements in.
         */
        // TODO(team): tune so pieces ingest without jamming
        public static final double INTAKE_POWER = 0.70;

        /**
         * Power used on the intake motor while reversing / outtaking.
         * Negative of intake by default so one sign convention is easy to flip later.
         */
        // TODO(team): tune so pieces eject cleanly
        public static final double OUTTAKE_POWER = -0.80;

        /**
         * Power used on the transfer motor while pulling game elements in
         * with the intake.
         */
        // TODO(team): tune transfer intake speed on the robot
        public static final double TRANSFER_INTAKE_POWER = 0.70;

        /**
         * Power used on the transfer motor while reversing / ejecting.
         */
        // TODO(team): tune transfer reverse speed on the robot
        public static final double TRANSFER_REVERSE_POWER = -0.80;

        /**
         * How far a trigger must be pressed before we treat it as held.
         *
         * <p>Triggers rest slightly above 0.0 on many gamepads, so a deadzone
         * prevents the intake from twitching when nobody is pressing.
         */
        public static final double TRIGGER_DEADZONE = 0.15;
    }

    /**
     * Hardware map names, flywheel RPM settings, and servo gate positions for
     * the outtake mechanism.
     *
     * <p>The outtake motor uses encoder velocity control to hold a target RPM,
     * and the servo gate uses positions in {@code [0.0, 1.0]}. The gate starts
     * closed at {@code init()} so game elements stay held until the driver
     * opens the gate.
     */
    public static final class Outtake {
        public static final String MOTOR = "outtake";
        public static final String GATE_SERVO = "servo";

        /**
         * Encoder ticks per output revolution. 28 is a bare goBILDA/REV
         * motor; multiply by the gear ratio if there is a gearbox.
         */
        // TODO(team): set for the real outtake motor / gearbox
        public static final double TICKS_PER_REV = 28.0;

        /**
         * Target flywheel RPM used while spinning up to score/eject pieces.
         */
        // TODO(team): tune target RPM so pieces score cleanly
        public static final double TARGET_RPM = 3800.0;

        /**
         * How close actual RPM must be to target to count as at speed.
         */
        public static final double RPM_TOLERANCE = 100.0;

        /**
         * Servo position when the gate is closed (blocking game pieces).
         * Used at init so the mechanism starts closed.
         */
        // TODO(team): measure closed gate position on the robot
        public static final double GATE_CLOSED_POSITION = 0.0;

        /**
         * Servo position when the gate is open (releasing game pieces).
         */
        // TODO(team): measure open gate position on the robot
        public static final double GATE_OPEN_POSITION = 1.0;
    }

    /**
     * Two-position servo constants retained as aliases for the outtake gate.
     */
    public static final class Servo {
        public static final String NAME = Outtake.GATE_SERVO;
        public static final double POSITION_A = Outtake.GATE_CLOSED_POSITION;
        public static final double POSITION_B = Outtake.GATE_OPEN_POSITION;
    }

    /**
     * Optional "drive here" pose used by TeleOp automation and autonomous
     * waypoints for Pedro Pathing.
     *
     * <p>Pedro reports position in inches and heading in radians (+X forward,
     * +Y left, counterclockwise positive). These defaults are placeholders so
     * the code compiles; they are not real field locations.
     */
    public static final class Field {
        // TODO(team): replace with measured field poses after localization is tuned
        public static final Pose HOME_POSE = new Pose(0.0, 0.0, Math.toRadians(0.0));
        public static final Pose START_POSE = new Pose(0.0, 0.0, Math.toRadians(0.0));
        public static final Pose PICKUP_POSE = new Pose(24.0, 0.0, Math.toRadians(0.0));
        public static final Pose SCORE_POSE = new Pose(24.0, 24.0, Math.toRadians(90.0));
        public static final Pose PARK_POSE = new Pose(48.0, 24.0, Math.toRadians(90.0));
    }
}
