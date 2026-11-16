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
 *    <li>Four mecanum drive motors: {@code left_front}, {@code right_front},
 *       {@code left_back}, {@code right_back}</li>
 *    <li>One intake {@link com.qualcomm.robotcore.hardware.DcMotor}: {@code intake}</li>
 *    <li>One transfer {@link com.qualcomm.robotcore.hardware.DcMotor}: {@code transfer}</li>
 *    <li>Two outtake {@link com.qualcomm.robotcore.hardware.DcMotor}s:
 *       {@code outtake_left}, {@code outtake_right}</li>
 *    <li>Two turret {@link com.qualcomm.robotcore.hardware.CRServo}s:
 *       {@code turretServoLeft}, {@code turretServoRight}, plus a REV Through Bore
 *       encoder named {@code turretEncoder}</li>
 *    <li>One two-position {@link com.qualcomm.robotcore.hardware.Servo}: {@code flower}</li>
 *    <li>goBILDA Pinpoint localizer named {@code pinpoint}</li>
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
         * Encoder ticks per output revolution for the transfer motor.
         */
        // TODO(team): set for the real transfer motor / gearbox
        public static final double TRANSFER_TICKS_PER_REV = 28.0;

        /**
         * Target velocity in RPM for the transfer motor when using encoder velocity control.
         */
        // TODO(team): tune transfer target RPM on the robot
        public static final double TRANSFER_TARGET_RPM = 5000.0;

        /**
         * Velocity PIDF coefficients applied to the transfer motor in
         * {@code RUN_USING_ENCODER} mode via {@code DcMotorEx.setVelocityPIDFCoefficients()}.
         */
        // TODO(team): tune transfer velocity PIDF gains on the robot
        public static final double TRANSFER_KP = 50.0;
        public static final double TRANSFER_KI = 0.0;
        public static final double TRANSFER_KD = 0.5;
        public static final double TRANSFER_KF = 10.0;

        /**
         * Power used on the intake motor while pulling game elements in.
         */
        // TODO(team): tune so pieces ingest without jamming
        public static final double INTAKE_POWER = 1;

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
        public static final double TRANSFER_INTAKE_POWER = 1;

        /**
         * Power used on the transfer motor while reversing / ejecting.
         */
        // TODO(team): tune transfer reverse speed on the robot
        public static final double TRANSFER_REVERSE_POWER = -.8;

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
         * Fixed flywheel RPM used whenever the outtake is spun up.
         */
        // TODO(team): tune so pieces score cleanly
        public static final double TARGET_RPM = 2450;
        /**
         * How close actual RPM must be to target to count as at speed.
         */
        public static final double RPM_TOLERANCE = 100.0;

        /**
         * Velocity PIDF coefficients applied to the outtake motors in
         * {@code RUN_USING_ENCODER} mode via {@code DcMotorEx.setVelocityPIDFCoefficients()}.
         *
         * <p>For a flywheel, feedforward ({@code F}) does most of the work to hold
         * steady-state RPM, and proportional ({@code P}) helps recover speed quickly
         * after a game piece is launched.
         *
         * <p>Each motor has its own set so the two flywheels can be tuned separately.
         */
        // TODO(team): tune velocity PIDF gains on the robot (use "Test: Motor PIDF Finder")
        public static final double LEFT_KP = 173.0;
        public static final double LEFT_KI = 0.0;
        public static final double LEFT_KD = 1.0;
        public static final double LEFT_KF = 10.0;

        public static final double RIGHT_KP = 173.0;
        public static final double RIGHT_KI = 0.0;
        public static final double RIGHT_KD = 1.0;
        public static final double RIGHT_KF = 10.0;

        /**
         * Gate servo that holds game pieces back from the flywheels. It is
         * opened manually (Gamepad 2 left trigger in TeleOp) and is closed
         * otherwise.
         */
        public static final String GATE_SERVO = "outtake_gate";
        // TODO(team): set both gate positions on the robot (0.0 to 1.0)
        public static final double GATE_CLOSED_POSITION = 0;
        public static final double GATE_OPEN_POSITION = .212;
    }


    /**
     * goBILDA Gear-Driven Turret (2.75:1, 176T ring gear, 64T drive gears) driven by
     * two continuous-rotation servos and measured by a REV Through Bore Encoder V2.
     * The turret rotates continuously; there are no cable limits.
     */
    public static final class Turret {
        // Hardware map names (must match the Robot Controller config exactly).
        public static final String SERVO_LEFT = "turretServoLeft";
        public static final String SERVO_RIGHT = "turretServoRight";

        /** REV Through Bore Encoder V2, configured as a motor encoder port. */
        public static final String ENCODER_PORT = "turretEncoder";

        /** Quadrature counts per revolution of the REV Through Bore Encoder V2. */
        public static final double ENCODER_TICKS_PER_REV = 8192.0;

        /**
         * Encoder shaft turns per turret turn. 64T drive gear on a 176T ring gear
         * is {@code 64/176} (1/2.75). Change the numerator if the encoder pinion
         * tooth count is not 64.
         */
        public static final double ENCODER_TO_TURRET_RATIO = 64.0 / 176.0;

        /** Set true if counterclockwise turret rotation produces negative ticks. */
        public static final boolean ENCODER_REVERSED = false;

        /**
         * Flip so BOTH servos push the turret the same direction, and so positive power
         * turns the turret counterclockwise (+angle). Test each servo alone at low power
         * first; if they fight each other they can burn out.
         *
         * <p>Both servos are mounted facing the same way on the same side of the turret,
         * and both pinions mesh with the same turret gear, so they must spin the same
         * way to move it. The two flags must therefore always MATCH (both false or both
         * true). If the turret turns away from the target instead of toward it, flip
         * BOTH flags together, never just one.
         */
        // TODO(team): confirm the turret turns toward the target on the robot
        public static final boolean SERVO_LEFT_REVERSED = false;
        public static final boolean SERVO_RIGHT_REVERSED = false;

        /** Servo turns per one turret turn, fixed by the 2.75:1 gear-driven turret. */
        public static final double GEAR_RATIO = 2.75;

        /**
         * Fastest target motion (rad/s) the velocity feedforward is allowed to chase.
         * 8.76 rad/s is 83.63 RPM at the turret for two goBILDA Super Speed servos.
         * Use 4.23 if the turret is driven by Axon MINI servos instead.
         */
        public static final double MAX_TARGET_VELOCITY_RAD_SEC = 8.76;

        /**
         * Smallest power sent to the servos while the turret is outside
         * {@link #ANGLE_TOLERANCE_RADIANS}, so a small error still overcomes stiction.
         */
        public static final double MIN_COMMAND_POWER = 0.15;

        /**
         * Time constant (seconds) of the low-pass filter on the target's angular velocity,
         * which feeds {@link #KV}. Larger is smoother but lags more; 0 turns the filter off.
         */
        public static final double TARGET_VELOCITY_FILTER_SECONDS = 0.10;

        /**
         * Turret pivot offset from the robot's localization center in inches
         * (+X is robot forward, +Y is robot left).
         */
        // TODO(team): measure turret pivot offset from robot center in inches
        public static final double OFFSET_X_INCHES = 0.0;
        public static final double OFFSET_Y_INCHES = 0.0;

        /**
         * The two hive sides in Pedro Pathing inches. The turret aims at the TOP side when the
         * robot is above SIDE_SWITCH_Y_INCHES and at the BOTTOM side when it is below.
         *
         * <p>Frame: the same one as {@link Field#START_POSE} and the autonomous paths, with
         * the origin at a field corner and both axes running 0 to 144 inches (NOT the
         * center-origin -72 to 72 frame). Every turret target must be given in this frame.
         */
        // TODO(team): replace these placeholders with the real hive coordinates (read them in the Pedro visualizer)
        public static final double TOP_TARGET_X_INCHES = 57.0;
        public static final double TOP_TARGET_Y_INCHES = 84.0;
        public static final double BOTTOM_TARGET_X_INCHES = 57.0;
        public static final double BOTTOM_TARGET_Y_INCHES = 60.0;

        /** Y position (inches) where the aimed side switches; 72 is the middle of a 144-inch field. */
        public static final double SIDE_SWITCH_Y_INCHES = 72.0;

        /** Dead band (inches) around the switch line so the target does not flicker. */
        public static final double SIDE_HYSTERESIS_INCHES = 3.0;

        /** Target used before the first pose arrives (the top side). */
        public static final double DEFAULT_TARGET_X_INCHES = BOTTOM_TARGET_X_INCHES;
        public static final double DEFAULT_TARGET_Y_INCHES = BOTTOM_TARGET_Y_INCHES;

        /** Secondary target coordinates, selected with Gamepad 2 Dpad Down in TeleOp. */
        // TODO(team): set secondary target X and Y coordinates in inches
        public static final double CENTER_TARGET_X_INCHES = 72.0;
        public static final double CENTER_TARGET_Y_INCHES = 72.0;

        /** Chassis-relative angle (radians) the turret returns to in HOME mode. */
        public static final double HOME_ANGLE_RADIANS = 0.0;

        /**
         * Chassis-relative angle (radians) the turret turns to on the first press of
         * Gamepad 2 LB; the next press returns it to {@link #HOME_ANGLE_RADIANS}.
         * Positive is counterclockwise (left); use -90 to turn right instead.
         */
        public static final double SIDE_ANGLE_RADIANS = Math.toRadians(90.0);

        /** Servo power cap in [0, 1]. Start low (0.5) for first tests, raise after tuning. */
        // TODO(team): raise once the turret behaves
        public static final double MAX_POWER = 0.9;

        /**
         * Controller gains. Output is servo power per radian of error. The through-bore
         * encoder is the angle feedback.
         *
         * <p>KF is the static breakaway feedforward: it is added in the direction of the
         * error whenever the turret is outside tolerance, so the proportional term works
         * on top of the power needed just to start moving.
         *
         * <p>KV is the velocity feedforward (power per rad/s of target motion), sized so
         * full turret speed takes about full servo power.
         */
        // TODO(team): tune on the robot
        public static final double KP = 1.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KV = 1.0 / MAX_TARGET_VELOCITY_RAD_SEC;
        public static final double KF = 0.08;

        /** Maximum integral accumulation (radians * seconds). */
        public static final double MAX_INTEGRAL = 0.5;

        /** Angle error deadband (radians) within which the turret counts as aimed. */
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
        public static final double POSITION_A = 0.3;

        /**
         * Second pose.
         */
        // TODO(team): measure this on the robot
        public static final double POSITION_B = 1;
    }

    /**
     * Timings for the autonomous "shoot balls" command in {@code MainAuto}.
     */
    public static final class Auto {
        /** Longest time to wait for the flywheels to reach speed before shooting anyway. */
        // TODO(team): tune on the robot
        public static final double SPIN_UP_TIMEOUT_MS = 3000.0;

        /** Time given to the gate servo to finish opening before balls are fed. */
        // TODO(team): tune on the robot
        public static final double GATE_OPEN_WAIT_MS = 300.0;

        /** How long the intake and transfer run to feed every ball into the flywheels. */
        // TODO(team): tune on the robot (long enough for all balls to leave)
        public static final double FEED_TIME_MS = 0500.0;
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
        public static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90.0));
        public static final Pose OPPOSE_POSE = new Pose(56, 136, Math.toRadians(270.0));
        public static final Pose PICKUP_POSE = new Pose(24.0, 0.0, Math.toRadians(0.0));
        public static final Pose SCORE_POSE = new Pose(24.0, 24.0, Math.toRadians(90.0));
        public static final Pose PARK_POSE = new Pose(48.0, 24.0, Math.toRadians(90.0));
    }
}