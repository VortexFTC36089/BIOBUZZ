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
 *   <li>Two outtake {@link com.qualcomm.robotcore.hardware.DcMotor}s:
 *       {@code outtake_left}, {@code outtake_right}</li>
 *   <li>One turret {@link com.qualcomm.robotcore.hardware.DcMotor}: {@code turretMotor}</li>
 *   <li>One two-position {@link com.qualcomm.robotcore.hardware.Servo}: {@code flower}</li>
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
         * Starting speed multiplier and step size when adjusting drive speed
         * with D-pad Up / D-pad Down.
         */
        public static final double DEFAULT_SPEED_MULTIPLIER = 1.00;
        public static final double SPEED_STEP = 0.10;
        public static final double MIN_SPEED_MULTIPLIER = 0.20;
        public static final double MAX_SPEED_MULTIPLIER = 1.00;

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
        public static final double INTAKE_POWER = 0.60;

        /**
         * Power used on the intake motor while reversing / outtaking.
         * Negative of intake by default so one sign convention is easy to flip later.
         */
        // TODO(team): tune so pieces eject cleanly
        public static final double OUTTAKE_POWER = -0.60;

        /**
         * Power used on the transfer motor while pulling game elements in
         * with the intake.
         */
        // TODO(team): tune transfer intake speed on the robot
        public static final double TRANSFER_INTAKE_POWER = 0.90;

        /**
         * Power used on the transfer motor while reversing / ejecting.
         */
        // TODO(team): tune transfer reverse speed on the robot
        public static final double TRANSFER_REVERSE_POWER = -0.70;

        /**
         * How far a trigger must be pressed before we treat it as held.
         *
         * <p>Triggers rest slightly above 0.0 on many gamepads, so a deadzone
         * prevents the intake from twitching when nobody is pressing.
         */
        public static final double TRIGGER_DEADZONE = 0.15;
    }

    /**
     * Hardware map names and flywheel RPM settings for the dual-motor outtake
     * mechanism.
     *
     * <p>Both outtake motors use encoder velocity control to hold a target RPM.
     */
    public static final class Outtake {
        public static final String LEFT_MOTOR = "outtake_left";
        public static final String RIGHT_MOTOR = "outtake_right";

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
        public static final double TARGET_RPM = 2300.0;

        /**
         * How close actual RPM must be to target to count as at speed.
         */
        public static final double RPM_TOLERANCE = 100.0;

        /**
         * Velocity PIDF coefficients applied to the outtake motor in
         * {@code RUN_USING_ENCODER} mode via {@code DcMotorEx.setVelocityPIDFCoefficients()}.
         *
         * <p>For a flywheel, feedforward ({@code F}) does most of the work to hold
         * steady-state RPM, and proportional ({@code P}) helps recover speed quickly
         * after a game piece is launched.
         */
        // TODO(team): tune velocity PIDF gains on the robot
        public static final double KP = 50.0;
        public static final double KI = 0.0;
        public static final double KD = 1.0;
        public static final double KF = 10.0;
    }

    /**
     * Hardware map name, geometry offsets, encoder resolution, rotation limits,
     * and software PIDF / velocity feedforward gains for the goBILDA Gear-Driven
     * Turret (SKU 3208-0004-0001) driven by a 435 RPM 5203 Series Yellow Jacket Motor.
     */
    public static final class Turret {
        public static final String MOTOR = "turretMotor";

        /**
         * goBILDA Gear-Driven Turret (SKU 3208-0004-0001) + 5203 Series 435 RPM Motor:
         * <ul>
         *   <li>Motor Encoder Resolution: {@code 383.6} ticks/rev at motor output shaft</li>
         *   <li>External Gear Reduction: {@code 2.75:1} (64T Hub-Mount Gear)</li>
         *   <li>Total Turret Resolution: {@code 383.6 * 2.75 = 1054.9} ticks/rev</li>
         *   <li>Ticks per Radian: {@code 1054.9 / (2 * PI) ≈ 167.89} ticks/rad</li>
         *   <li>Ticks per Degree: {@code 1054.9 / 360.0 ≈ 2.93} ticks/deg</li>
         *   <li>Max Turret Speed: {@code 435 / 2.75 = 158.18 RPM} ({@code 16.564 rad/s})</li>
         * </ul>
         */
        public static final double MOTOR_TICKS_PER_REV = 383.6;
        public static final double GEAR_RATIO = 2.75;
        public static final double TICKS_PER_TURRET_REV = 1054.9;
        public static final double TICKS_PER_RADIAN = 167.89;
        public static final double TICKS_PER_DEGREE = 2.93;
        public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 16.564;

        /**
         * Turret pivot offset from the robot's localization center in inches
         * (+X is robot forward, +Y is robot left).
         */
        // TODO(team): measure turret pivot offset from robot center in inches
        public static final double OFFSET_X_INCHES = 0.0;
        public static final double OFFSET_Y_INCHES = 0.0;

        /**
         * Default (High Goal) field target coordinates in Pedro Pathing inches.
         */
        // TODO(team): set default goal/target X and Y coordinates in inches
        public static final double DEFAULT_TARGET_X_INCHES = 57.0;
        public static final double DEFAULT_TARGET_Y_INCHES = 72.0;

        /**
         * Secondary (center / submersible) field target coordinates in Pedro
         * Pathing inches, selected with Gamepad 2 B in TeleOp.
         */
        // TODO(team): set secondary target X and Y coordinates in inches
        public static final double CENTER_TARGET_X_INCHES = 72.0;
        public static final double CENTER_TARGET_Y_INCHES = 72.0;

        /**
         * Chassis-relative angle (radians) the turret returns to when the
         * driver requests the home / starting position.
         */
        public static final double HOME_ANGLE_RADIANS = 0.0;

        /**
         * Whether soft rotation limits are enforced to prevent wire binding.
         */
        public static final boolean USE_SOFT_LIMITS = true;

        /**
         * Minimum and maximum allowed turret angles relative to the robot
         * chassis (0° = straight forward, positive = counterclockwise).
         */
        // TODO(team): measure safe mechanical rotation limits in degrees
        public static final double MIN_ANGLE_DEGREES = -180.0;
        public static final double MAX_ANGLE_DEGREES = 180.0;
        public static final double MIN_ANGLE_RADIANS = Math.toRadians(MIN_ANGLE_DEGREES);
        public static final double MAX_ANGLE_RADIANS = Math.toRadians(MAX_ANGLE_DEGREES);

        /**
         * Software positional PIDF + Velocity Feedforward (kV) gains:
         * <ul>
         *   <li>{@code KP = 1.2}</li>
         *   <li>{@code KI = 0.0}</li>
         *   <li>{@code KD = 0.05}</li>
         *   <li>{@code KV = 0.0604} ({@code 1.0 / 16.564 rad/s})</li>
         *   <li>{@code KF = 0.0}</li>
         * </ul>
         */
        // TODO(team): tune turret software PIDF/kV gains on the robot
        public static final double KP = 1.2;
        public static final double KI = 0.0;
        public static final double KD = 0.05;
        public static final double KV = 0.0604;
        public static final double KF = 0.0;

        /**
         * Maximum integral accumulation (radians * seconds) to prevent integral windup.
         */
        public static final double MAX_INTEGRAL = 0.5;

        /**
         * Angle error deadband (radians) within which the turret is considered aimed.
         */
        public static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(1.0);
    }

    /**
     * Two-position servo (claw, hood, flap, etc.).
     *
     * <p>Servo positions are unitless in {@code [0.0, 1.0]}. 0.0 and 1.0
     * are the mechanical ends of travel; measure the two poses you actually
     * want and put them here.
     */
    public static final class Servo {
        public static final String NAME = "flower";

        /**
         * First pose. Used at init so the mechanism starts in a known place.
         */
        // TODO(team): measure this on the robot
        public static final double POSITION_A = 0.0;

        /**
         * Second pose.
         */
        // TODO(team): measure this on the robot
        public static final double POSITION_B = .5;
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
