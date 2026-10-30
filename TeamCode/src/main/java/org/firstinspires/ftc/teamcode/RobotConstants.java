package org.firstinspires.ftc.teamcode;

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
 *   <li>One two-position servo named {@code servo}</li>
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
     * Hardware map name and power values for the single-motor intake.
     *
     * <p>Powers are unitless motor commands in {@code [-1.0, 1.0]}. Positive
     * vs negative is a hardware convention; we treat positive as "intake"
     * until the team verifies the motor wiring.
     */
    public static final class Intake {
        public static final String MOTOR = "intake";

        /**
         * Power used while pulling game elements in.
         */
        // TODO(team): tune so pieces ingest without jamming
        public static final double INTAKE_POWER = 0.70;

        /**
         * Power used while reversing / outtaking. Negative of intake by
         * default so one sign convention is easy to flip later.
         */
        // TODO(team): tune so pieces eject cleanly
        public static final double OUTTAKE_POWER = -0.80;

        /**
         * How far a trigger must be pressed before we treat it as held.
         *
         * <p>Triggers rest slightly above 0.0 on many gamepads, so a deadzone
         * prevents the intake from twitching when nobody is pressing.
         */
        public static final double TRIGGER_DEADZONE = 0.15;
    }

    /**
     * Two-position servo (claw, hood, flap, etc.).
     *
     * <p>Servo positions are unitless in {@code [0.0, 1.0]}. 0.0 and 1.0
     * are the mechanical ends of travel; measure the two poses you actually
     * want and put them here.
     */
    public static final class Servo {
        public static final String NAME = "servo";

        /**
         * First pose. Used at init so the mechanism starts in a known place.
         */
        // TODO(team): measure this on the robot
        public static final double POSITION_A = 0.0;

        /**
         * Second pose.
         */
        // TODO(team): measure this on the robot
        public static final double POSITION_B = 1.0;
    }

    /**
     * Optional "drive here" pose used by TeleOp automation.
     *
     * <p>Pedro reports position in inches and heading in radians. These
     * defaults are placeholders so the code compiles; they are not a real
     * field location.
     */
    public static final class Field {
        // TODO(team): replace with a measured field pose after localization is tuned
        public static final double HOME_X_INCHES = 0.0;
        public static final double HOME_Y_INCHES = 0.0;
        public static final double HOME_HEADING_RADIANS = 0.0;
    }

    /**
     * Hive (goal) positions and the heading lock that aims the chassis at them.
     *
     * <p>Pedro field coordinates: inches, +X forward, +Y left. There is one
     * hive on each side of the field. When the robot's Y is at or above
     * {@link #FIELD_MIDLINE_Y_INCHES} it aims at the left hive, otherwise the
     * right hive.
     */
    public static final class Hive {
        // TODO(team): measure both hive centers in Pedro coordinates
        public static final double LEFT_HIVE_X_INCHES = 132.0;
        public static final double LEFT_HIVE_Y_INCHES = 132.0;
        public static final double RIGHT_HIVE_X_INCHES = 132.0;
        public static final double RIGHT_HIVE_Y_INCHES = 12.0;

        /**
         * Y value that splits the field into the left and right halves.
         */
        // TODO(team): set to the real midline (72 on a 0-144 field)
        public static final double FIELD_MIDLINE_Y_INCHES = 72.0;

        /**
         * Added to the robot-to-hive angle. 0 if the shooter fires out the
         * front of the robot, {@code Math.PI} if it fires out the back.
         */
        public static final double SHOOTER_HEADING_OFFSET_RADIANS = 0.0;

        /**
         * Turn power per radian of heading error while aiming.
         *
         * <p>If the robot spins away from the hive instead of toward it, make
         * this negative.
         */
        // TODO(team): tune; raise until it snaps on, lower if it oscillates
        public static final double AIM_KP = 1.2;

        /**
         * Damping on the heading lock. Raise a little if the robot overshoots
         * and wobbles around the hive; leave at 0 if it settles cleanly.
         */
        // TODO(team): tune after AIM_KP
        public static final double AIM_KD = 0.0;

        /**
         * Most turn power the heading lock may command, in {@code [0, 1]}.
         */
        public static final double AIM_MAX_TURN = 0.7;

        /**
         * Heading error under which the robot counts as aimed.
         */
        public static final double AIM_TOLERANCE_RADIANS = Math.toRadians(2.0);
    }

    /**
     * Flywheel shooter that picks its speed from the distance to the hive.
     */
    public static final class Shooter {
        public static final String MOTOR = "shooter";

        /**
         * Encoder ticks per output revolution. 28 is a bare goBILDA/REV
         * motor; multiply by the gear ratio if there is a gearbox.
         */
        // TODO(team): set for the real shooter motor
        public static final double TICKS_PER_REV = 28.0;

        /**
         * Distance-to-RPM table. Distances are inches from robot center to
         * hive center, ascending. RPM is linearly interpolated between rows
         * and clamped at both ends.
         *
         * <p>The first row is the base shot: robot against the wall,
         * perpendicular to the hive, at 3800 RPM.
         *
         * <p>To tune: park at each distance, adjust that row's RPM until shots
         * land, then add rows where shots miss in between.
         */
        // TODO(team): replace 24.0 with the "Hive distance (in)" telemetry
        //  reading while parked at the wall spot, then tune the farther rows
        public static final double[] DISTANCES_INCHES = {24.0, 48.0, 72.0, 96.0, 120.0};
        public static final double[] RPMS = {3800.0, 4100.0, 4400.0, 4700.0, 5000.0};

        /**
         * How close actual RPM must be to target to count as ready to shoot.
         */
        public static final double RPM_TOLERANCE = 100.0;
    }
}
