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
 *       Encoder V2 plugged into the encoder socket of the {@code intake} motor port
 *       (all eight motor ports are in use, so it has no config name of its own)</li>
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
     * Driver Station names of the match OpModes, and which TeleOp follows autonomous.
     *
     * <p>Every OpMode name must be different. If two OpModes share a name the SDK renames
     * both, and one of them stops being listed as Autonomous and loses its preselected
     * TeleOp.
     */
    public static final class OpModeNames {
        /** Name of {@link MainTeleOp} on the Driver Station. */
        public static final String MAIN_TELEOP = "Main TeleOp";

        /**
         * The TeleOp the Driver Station switches to by itself when an autonomous ends.
         * CHANGE THIS ONE LINE to send both autonomous programs to a different TeleOp.
         * It must be exactly the {@code name} of an OpMode annotated {@code @TeleOp}
         * (for example {@code "Test: Intake"}); if it matches nothing, no TeleOp is
         * preselected.
         *
         * <p>The Driver Station selects that TeleOp for you. A driver still has to press
         * Init and then Play: the Robot Controller only starts an OpMode when the Driver
         * Station tells it to, so code cannot start TeleOp. Check the switch once on your
         * own Driver Station: run an autonomous, let it end, and confirm this TeleOp is
         * the one selected.
         */
        public static final String TELEOP_AFTER_AUTO = MAIN_TELEOP;

        /** {@link MainAuto}: starts at the bottom of the field, (56, 8) facing 90 degrees. */
        public static final String AUTO_BOTTOM = "AutoPath Bottom";

        /** {@link SecondAuto}: starts at the top of the field, (56, 136) facing 270 degrees. */
        public static final String AUTO_TOP = "AutoPath Top";
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
     * goBILDA Gear-Driven Turret (2.75:1 Ratio, 105mm ID) driven by TWO continuous-rotation
     * servos and measured by a REV Through Bore Encoder V2 mounted on one servo's output
     * shaft. Both servos sit on the same side of the turret and their pinions mesh with the
     * same turret gear. The turret angle is read from the encoder, so it does not drift.
     */
    public static final class Turret {
        // Hardware map names (must match the Robot Controller config exactly).
        public static final String SERVO_LEFT = "turretServoLeft";
        public static final String SERVO_RIGHT = "turretServoRight";

        /**
         * Config name of the MOTOR whose encoder socket the REV Through Bore Encoder V2
         * is plugged into.
         *
         * <p>The robot already uses all eight motor ports (four drive, intake, transfer,
         * two outtake), so the turret encoder cannot have a port or a config name of its
         * own. Its 4-pin quadrature cable goes into the ENCODER socket next to a motor that
         * does not use its own encoder, and the code reads the count through that motor.
         * The intake motor is the right one: it runs without its encoder. Do NOT use the
         * transfer or outtake ports (they run on their encoders).
         * REV recommends hub motor ports 0 or 3 for this high-resolution encoder.
         *
         * <p>If the cable is in a different motor's encoder socket, put THAT motor's config
         * name here. The turret code only reads the count; it never changes that motor's
         * mode, power or direction.
         */
        // TODO(team): confirm the encoder cable is in the intake motor's encoder socket
        public static final String ENCODER_PORT = Intake.MOTOR;

        /** Quadrature counts per revolution of the REV Through Bore Encoder V2 (2048 CPR x 4). */
        public static final double ENCODER_TICKS_PER_REV = 8192.0;

        /** Servo turns per one turret turn, fixed by the 2.75:1 gear-driven turret. */
        public static final double GEAR_RATIO = 2.75;

        /**
         * Turret turns per ONE encoder turn. The encoder rides on a servo output shaft, and
         * the servo pinion turns {@link #GEAR_RATIO} times per turret turn, so this is
         * {@code 1 / GEAR_RATIO}. If the encoder is ever moved onto the turret ring itself,
         * set this to 1.0. Check: one full turret turn by hand must change
         * {@code Turret encoderTicks} by 22528.
         */
        public static final double ENCODER_TO_TURRET_RATIO = 1.0 / GEAR_RATIO;

        /**
         * Set true if turning the turret counterclockwise (looking down from above, the
         * +angle direction) makes {@code Turret measuredAngleDeg} go DOWN. A pinion on a
         * ring gear spins opposite to the ring, so this often needs to be true.
         *
         * <p>Check it in INIT of Main TeleOp, before Play: the turret servos are unpowered
         * there and the init screen shows the live angle and ticks, so turn the turret by
         * hand and watch. The sign also depends on the motor TYPE chosen for the shared
         * port in the Robot Controller config (some types, such as goBILDA 5202 and REV HD
         * Hex, are counted in reverse), so re-check this flag after changing the type. The
         * intake motor's own FORWARD/REVERSE setting does not matter: TurretSubsystem
         * cancels it out.
         */
        // TODO(team): confirm on the robot
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

        /**
         * No-load speed of ONE turret servo at full power, in RPM, from its spec sheet.
         * Spec sheets usually give seconds per 60 degrees: RPM = 10 / (seconds per 60 degrees).
         * Use the figure for the voltage the servos actually run at.
         */
        // 230 servo RPM / 2.75 = 83.63 RPM at the turret (goBILDA Super Speed, near no load).
        // Do not lower this back to a placeholder: too low makes the turret overshoot.
        public static final double SERVO_FREE_SPEED_RPM = 230.0;

        /**
         * Turret speed (radians/second) at power 1.0, from the servo speed and the gear
         * ratio. The encoder measures the real angle, so this no longer affects aiming
         * accuracy. It only caps how fast a target may appear to move for the {@link #KV}
         * feedforward and sizes {@link #KV} itself.
         */
        public static final double RAD_PER_SEC_AT_FULL_POWER =
                SERVO_FREE_SPEED_RPM * 2.0 * Math.PI / 60.0 / GEAR_RATIO;

        /**
         * Servo power at or below which the turret does NOT move (servo dead zone plus
         * turret friction). Used to size {@link #MIN_COMMAND_POWER} and {@link #KF}.
         *
         * <p>To measure: command the turret servos at a fixed low power and raise it in
         * small steps until the turret just starts to turn in both directions; use the
         * larger of the two values.
         */
        // TODO(team): measure on the robot (0.08 is only a placeholder)
        public static final double SERVO_DEADBAND_POWER = 0.08;

        /**
         * Smallest power used for a correction TOWARD the target while the turret is outside
         * {@link #ANGLE_TOLERANCE_RADIANS}. Must stay above {@link #SERVO_DEADBAND_POWER} so
         * the correction actually moves the turret. While the target is moving, weaker (or
         * zero) power can still be sent: feedforward in the other direction is left alone,
         * and a turret that is ahead of a target coming toward it just waits. Raising this
         * makes the turret step in bigger jumps and it can start to hunt back and forth.
         */
        public static final double MIN_COMMAND_POWER = SERVO_DEADBAND_POWER + 0.07;

        /**
         * Time constant (seconds) of the low-pass filter on the target's angular velocity,
         * which feeds {@link #KV}. Larger is smoother but lags more; 0 turns the filter off.
         */
        public static final double TARGET_VELOCITY_FILTER_SECONDS = 0.10;

        /**
         * Target angular speed (radians/second) above which the target counts as MOVING.
         * While it is moving, the turret keeps following with feedforward even inside
         * {@link #ANGLE_TOLERANCE_RADIANS} instead of stopping dead, so it does not
         * stop-start while the chassis spins. Below this speed the turret stops once aimed.
         */
        public static final double TARGET_MOVING_THRESHOLD_RAD_SEC = Math.toRadians(10.0);

        /**
         * Longest time (milliseconds) the TeleOp shoot sequence waits for the turret to
         * finish aiming before it fires anyway. Only applies while the turret is TRACKING.
         */
        public static final double AIM_WAIT_TIMEOUT_MS = 750.0;

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

        /** Target used before the first pose arrives (the bottom side). */
        public static final double DEFAULT_TARGET_X_INCHES = BOTTOM_TARGET_X_INCHES;
        public static final double DEFAULT_TARGET_Y_INCHES = BOTTOM_TARGET_Y_INCHES;

        /** Chassis-relative angle (radians) the turret returns to in HOME mode. */
        public static final double HOME_ANGLE_RADIANS = 0.0;

        /**
         * Chassis-relative angle (radians) the turret turns to on the first press of
         * Gamepad 2 LB; the next press returns it to {@link #HOME_ANGLE_RADIANS}.
         * Positive is counterclockwise (left); use -90 to turn right instead.
         */
        public static final double SIDE_ANGLE_RADIANS = Math.toRadians(90.0);

        /**
         * Whether soft rotation limits are enforced to prevent wire binding. {@code false}
         * means the turret spins without limit and always takes the shortest path to the
         * target. Set back to {@code true} if any cable can wrap around the turret.
         */
        public static final boolean USE_SOFT_LIMITS = false;

        /**
         * Allowed turret angles relative to the chassis (0 = forward, + = counterclockwise).
         * Only used while {@link #USE_SOFT_LIMITS} is {@code true}.
         */
        // TODO(team): measure safe mechanical rotation limits in degrees
        public static final double MIN_ANGLE_DEGREES = -180.0;
        public static final double MAX_ANGLE_DEGREES = 180.0;
        public static final double MIN_ANGLE_RADIANS = Math.toRadians(MIN_ANGLE_DEGREES);
        public static final double MAX_ANGLE_RADIANS = Math.toRadians(MAX_ANGLE_DEGREES);

        /**
         * Servo power cap in [0, 1], and the power used for the fast part of every move
         * (anything farther from the target than {@link #SLOWDOWN_START_DEGREES}).
         * Start low (0.3 to 0.5) for first tests.
         *
         * <p>The slow-down ramp counts DOWN from this power, so changing MAX_POWER shifts
         * the power at every distance inside the slow zone by the same amount. Re-tune
         * {@link #SLOWDOWN_START_DEGREES} and {@link #SLOWDOWN_POWER_PER_DEGREE} every time
         * this changes. At a low MAX_POWER the values tuned for 0.9 make the turret crawl
         * at minimum power for a long way (at 0.4 with 0.018 per degree, a
         * SLOWDOWN_START_DEGREES of about 17 instead of 45 removes the crawl).
         */
        // TODO(team): raise in steps of 0.1 once the turret behaves, re-tuning the slow-down each time
        public static final double MAX_POWER = 0.9;

        /**
         * TURNING SPEED PROFILE, knob 1 of 2: how far from the target (degrees) the turret
         * STARTS slowing down.
         *
         * <p>Farther from the target than this, the turret runs flat out at
         * {@link #MAX_POWER}. Once it is closer than this, the power is reduced by
         * {@link #SLOWDOWN_POWER_PER_DEGREE} for every degree it gets closer, but never
         * below {@link #MIN_COMMAND_POWER}. Inside {@link #ANGLE_TOLERANCE_RADIANS} it stops.
         *
         * <p>Tuning: if the turret OVERSHOOTS the target and comes back, make this BIGGER so
         * it starts braking earlier. If it slows down too early and wastes time, make it
         * smaller.
         *
         * <p>The two knobs are linked. The power only gets down to the minimum before the
         * target if {@code SLOWDOWN_START_DEGREES * SLOWDOWN_POWER_PER_DEGREE} is at least
         * {@code MAX_POWER - MIN_COMMAND_POWER} (0.75 with the values here; 45 x 0.018 is
         * 0.81). A turret that arrives above the minimum power does not stop: it swings
         * back and forth across the target. So when this is made smaller the code makes
         * the slow-down steeper by itself to keep that from happening (see
         * {@link #SLOWDOWN_POWER_PER_DEGREE}); a very small zone therefore means very
         * hard braking, and a heavy turret will still overshoot it.
         */
        // TODO(team): tune on the robot
        public static final double SLOWDOWN_START_DEGREES = 45.0;

        /**
         * TURNING SPEED PROFILE, knob 2 of 2: how quickly the turret slows down once it is
         * inside {@link #SLOWDOWN_START_DEGREES}, as servo power removed per degree closer
         * to the target.
         *
         * <p>Power at a given distance inside the zone is
         * {@code MAX_POWER - SLOWDOWN_POWER_PER_DEGREE * (SLOWDOWN_START_DEGREES - distance)},
         * held at {@link #MIN_COMMAND_POWER} once it gets that low. With the values here
         * (0.9 max, 45 degrees, 0.018 per degree, 0.15 min) the power is 0.9 at 45 degrees,
         * 0.45 at 20 degrees, 0.27 at 10 degrees, and reaches the 0.15 minimum at about
         * 3.3 degrees, then creeps the rest of the way.
         *
         * <p>Tuning: a BIGGER number brakes harder, so the turret reaches the slow creep
         * sooner and travels the last part slowly (safe, but slower). A SMALLER number
         * brakes more gently and saves time.
         *
         * <p>There is a lowest value that can work:
         * {@code (MAX_POWER - MIN_COMMAND_POWER) / SLOWDOWN_START_DEGREES} (0.0167 with the
         * values here). That value reaches the minimum power exactly at the target. Below
         * it the turret would still be driving hard when it arrives and would swing back
         * and forth across the target without stopping, so the code never uses less than
         * that: a number smaller than the limit (or 0) simply behaves like the limit.
         * Staying about 10 percent above it, as 0.018 does, leaves a short creep at the end.
         */
        // TODO(team): tune on the robot
        public static final double SLOWDOWN_POWER_PER_DEGREE = 0.018;

        /**
         * Extra controller terms on top of the turning speed profile above. The profile
         * decides the power from the distance to the target; these add to it.
         *
         * <p>KI and KD are off (0). Add a little KD only if the turret still overshoots
         * after the two slow-down knobs are tuned. If the turret stops short of the target,
         * measure {@link #SERVO_DEADBAND_POWER} first (it is set too low); KI would also
         * fix it, but KI builds up during every long move and makes the turret overshoot more.
         *
         * <p>KF is the breakaway power: the power needed just to start the turret moving.
         * It is used when the turret is already aimed and only has to keep pace with a
         * moving target.
         *
         * <p>KV is the velocity feedforward (power per rad/s of target motion), sized so
         * full turret speed takes about full servo power. It lets the turret keep pace
         * with the target while the chassis spins instead of trailing behind it. Set to
         * 0 to turn it off.
         */
        // TODO(team): tune on the robot
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KV = (1.0 - SERVO_DEADBAND_POWER) / RAD_PER_SEC_AT_FULL_POWER;
        public static final double KF = SERVO_DEADBAND_POWER;

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
     * Timings for the "shoot balls" command in {@code SecondAuto} and {@code MainTeleOp}.
     */
    public static final class Auto {
        /** Longest time to wait for the flywheels to reach speed before shooting anyway. */
        // TODO(team): tune on the robot
        public static final double SPIN_UP_TIMEOUT_MS = 3000.0;

        /** Time given to the gate servo to finish opening before balls are fed. */
        // TODO(team): tune on the robot
        public static final double GATE_OPEN_WAIT_MS = 300.0;

        /**
         * How long the intake and transfer run to feed every ball into the flywheels.
         * Only time with the feed actually running counts: pauses that
         * {@link ShotFeeder} adds while the wheels recover are not included.
         */
        // TODO(team): tune on the robot (long enough for all balls to leave)
        public static final double FEED_TIME_MS = 0500.0;
    }

    /**
     * Rapid-fire feeding: how balls are pushed into the flywheels during a shot so that
     * every ball in a burst meets the wheels at the same speed and flies the same arc.
     * Used by {@link ShotFeeder} in Main TeleOp and in autonomous.
     *
     * <p>How it works: when a burst starts, the speed of each flywheel is remembered. The
     * intake and transfer then feed ONLY while both wheels are still within
     * {@link #RPM_TOLERANCE} of that speed. Each ball slows the wheels; when they drop out
     * of the band the feed pauses, and it resumes as soon as they have recovered. With
     * heavy flywheels the feed never pauses (full rapid fire); with light ones it pauses
     * for a moment between balls.
     *
     * <p>Tune with the "Burst ..." lines on the Driver Station after a shot.
     */
    public static final class ShotFeed {
        /**
         * Master switch. {@code false} turns the pausing off completely: the feed then
         * runs without stopping for {@link Auto#FEED_TIME_MS}, exactly as it did before
         * this was added. Use that if the new behaviour ever causes trouble at an event.
         */
        public static final boolean GATING_ENABLED = true;

        /**
         * How far (RPM) a flywheel may be from the speed it had when the burst started
         * before the feed pauses. Smaller = balls land closer together, but more pausing.
         *
         * <p>Do not go below about 50. The hub reports flywheel speed in coarse steps
         * (about 43 RPM for a 28-tick motor), so a tighter band pauses on measurement
         * noise alone.
         */
        // TODO(team): tune on the robot
        public static final double RPM_TOLERANCE = 60.0;

        /**
         * Loops in a row both flywheels must be at speed (within
         * {@link Outtake#RPM_TOLERANCE} of the target) before a burst may start, so one
         * lucky reading while still spinning up does not start it.
         */
        public static final int READY_LOOPS = 3;

        /**
         * Longest single pause (milliseconds). If the wheels have not recovered by then,
         * the feeder stops waiting and feeds the rest of the burst without pauses, so a
         * shot can never get stuck. If the Driver Station shows "GAVE UP WAITING" often,
         * the wheels need more than this to recover: raise it, or add flywheel weight.
         */
        // TODO(team): tune on the robot
        public static final double MAX_PAUSE_MS = 500.0;

        /** Hard limit (milliseconds) on a whole burst, pauses included. */
        public static final double MAX_TOTAL_MS = 4000.0;

        /**
         * Intake power while feeding a shot. Same as normal intaking for now. The intake
         * is not speed-controlled, so it slows as the battery drops.
         */
        // TODO(team): tune on the robot
        public static final double INTAKE_POWER = Intake.INTAKE_POWER;

        /**
         * Transfer speed (RPM) while feeding a shot. Same as normal transferring for now.
         *
         * <p>This sets the time between balls: time between balls = ball width / feed
         * speed. Slower = more time for the wheels to recover between balls. It must be a
         * speed the motor can really hold under load: compare it with "Transfer RPM while
         * feeding" on the Driver Station. If the measured number is well below this one,
         * the motor is flat out and the ball spacing changes with the battery, so lower
         * this until the two agree. Feeding slower moves the balls less far in the same
         * time, so raise {@link Auto#FEED_TIME_MS} to match.
         */
        // TODO(team): tune on the robot
        public static final double TRANSFER_RPM = Intake.TRANSFER_TARGET_RPM;

        /**
         * Extra flywheel target speed (RPM) while the feed is running, so the motors are
         * already pushing harder when a ball arrives instead of reacting after the speed
         * has dropped. 0 = off. Try small steps (20 to 50) only after everything else is
         * tuned: too much makes the first ball go long.
         */
        public static final double BOOST_RPM = 0.0;
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