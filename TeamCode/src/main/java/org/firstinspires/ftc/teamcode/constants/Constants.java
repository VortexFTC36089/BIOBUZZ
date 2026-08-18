package org.firstinspires.ftc.teamcode.constants;

/**
 * Central repository for hardware map names and tunable parameters.
 *
 * Hardware map names MUST match the Driver Hub robot configuration exactly.
 * If an OpMode crashes during INIT with a "Unable to find a hardware device"
 * error, the name in this file does not match the configuration on the robot.
 *
 * Values marked [TUNE] should be adjusted on the robot, not guessed.
 */
public final class Constants {

    private Constants() {}

    public static final class Drive {
        public static final String LEFT_FRONT  = "leftFront";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String LEFT_REAR   = "leftRear";
        public static final String RIGHT_REAR  = "rightRear";

        /** [TUNE] Maximum drive power in normal mode. */
        public static final double MAX_POWER = 1.0;

        /** [TUNE] Drive power scale while slow mode is held. */
        public static final double SLOW_MODE_FACTOR = 0.35;

        /** [TUNE] Joystick magnitude below which input is treated as zero. */
        public static final double STICK_DEADZONE = 0.05;

        /** [TUNE] Power used by Drivetrain Test for isolated motor checks. */
        public static final double TEST_POWER = 0.3;
    }

    public static final class Intake {
        public static final String MOTOR = "intake";

        /** [TUNE] Inward collection power. */
        public static final double INTAKE_POWER = 0.8;

        /** [TUNE] Reverse / expel power. */
        public static final double OUTTAKE_POWER = -0.6;

        /** Trigger magnitude required before intake commands apply. */
        public static final double TRIGGER_THRESHOLD = 0.1;
    }
}
