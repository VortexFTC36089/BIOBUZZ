package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


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
 *       Encoder V2 plugged into the encoder socket of the motor port named by
 *       {@code Turret.ENCODER_PORT} (all eight motor ports are in use, so it has no
 *       config name of its own)</li>
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
    @Configurable
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
        public static double SLOW_MODE_MULTIPLIER = 0.30;

        /**
         * Starting speed multiplier and step size when adjusting drive speed
         * with D-pad Up / D-pad Down.
         */
        public static double DEFAULT_SPEED_MULTIPLIER = 1.00;
        public static double SPEED_STEP = 0.10;
        public static double MIN_SPEED_MULTIPLIER = 0.20;
        public static double MAX_SPEED_MULTIPLIER = 1.00;

        /**
         * Stick magnitude that counts as "the driver is moving" when we cancel
         * an automated drive-to-pose.
         *
         * <p>A small deadzone avoids cancelling because of stick drift.
         */
        public static double AUTOMATION_CANCEL_DEADZONE = 0.15;
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
    @Configurable
    public static final class Intake {
        public static final String MOTOR = "intake";
        public static final String TRANSFER_MOTOR = "transfer";

        /**
         * Encoder ticks per output revolution for the transfer motor.
         */
        // TODO(team): set for the real transfer motor / gearbox
        public static double TRANSFER_TICKS_PER_REV = 28.0;

        /**
         * Target velocity in RPM for the transfer motor when using encoder velocity control.
         */
        // TODO(team): tune transfer target RPM on the robot
        public static double TRANSFER_TARGET_RPM = 1700;

        /**
         * Velocity PIDF coefficients applied to the transfer motor in
         * {@code RUN_USING_ENCODER} mode via {@code DcMotorEx.setVelocityPIDFCoefficients()}.
         */
        // TODO(team): tune transfer velocity PIDF gains on the robot
        public static double TRANSFER_KP = 40.0;
        public static double TRANSFER_KI = 0.0;
        public static double TRANSFER_KD = 0.5;
        public static double TRANSFER_KF = 10.0;

        /**
         * Power used on the intake motor while pulling game elements in.
         */
        // TODO(team): tune so pieces ingest without jamming
        public static double INTAKE_POWER = 1;

        /**
         * Power used on the intake motor while reversing / outtaking.
         * Negative of intake by default so one sign convention is easy to flip later.
         */
        // TODO(team): tune so pieces eject cleanly
        public static double OUTTAKE_POWER = -0.80;

        /**
         * Power used on the transfer motor while pulling game elements in
         * with the intake.
         */
        // TODO(team): tune transfer intake speed on the robot
        public static double TRANSFER_INTAKE_POWER = 1;

        /**
         * Power used on the transfer motor while reversing / ejecting.
         */
        // TODO(team): tune transfer reverse speed on the robot
        public static double TRANSFER_REVERSE_POWER = -.8;

        /**
         * How far a trigger must be pressed before we treat it as held.
         *
         * <p>Triggers rest slightly above 0.0 on many gamepads, so a deadzone
         * prevents the intake from twitching when nobody is pressing.
         */
        public static double TRIGGER_DEADZONE = 0.15;
    }

    /**
     * Hardware map names and flywheel RPM settings for the dual-motor outtake
     * mechanism.
     *
     * <p>Both outtake motors use custom voltage feedforward and proportional
     * velocity feedback to hold a target RPM.
     */
    @Configurable
    public static final class Outtake {
        public static final String LEFT_MOTOR = "outtake_left";
        public static final String RIGHT_MOTOR = "outtake_right";

        /**
         * Encoder ticks per flywheel revolution. 28 is the current assumption
         * for a bare goBILDA motor with no external reduction; verify it against
         * the exact motor's encoder specification before relying on RPM values.
         */
        // TODO(team): verify the SDK encoder ticks per revolution for the exact motor
        public static double TICKS_PER_REV = 28.0;

        /**
         * Fixed flywheel RPM used whenever the outtake is spun up.
         */
        // TODO(team): tune so pieces score cleanly
        public static double TARGET_RPM = 1950;
        /** Alternate target used by the outtake tuning OpMode; tune for the second shot mode. */
        public static double SECONDARY_TARGET_RPM = 1300;
        /**
         * How close actual RPM must be to target to count as at speed.
         */
        public static double RPM_TOLERANCE = 100.0;

        /**
         * Gate servo that holds game pieces back from the flywheels. It is
         * opened manually (Gamepad 2 left trigger in TeleOp) and is closed
         * otherwise.
         */
        public static final String GATE_SERVO = "outtake_gate";
        // TODO(team): set both gate positions on the robot (0.0 to 1.0)
        public static double GATE_CLOSED_POSITION = .58;
        public static double GATE_OPEN_POSITION = .72;
    }


    /**
     * goBILDA Gear-Driven Turret (2.75:1 Ratio, 105mm ID) driven by TWO continuous-rotation
     * servos and measured by a REV Through Bore Encoder V2 geared to the turret ring (on
     * its own small gear, see {@link #ENCODER_TO_TURRET_RATIO}). Both servos sit on the
     * same side of the turret and their pinions mesh with the same turret gear. The turret
     * angle is read from the encoder, so it does not drift.
     *
     * <p>Tunable from Panels. This class is {@code @Configurable} and every tunable value is
     * a plain (non-final) static field that {@link TurretSubsystem} reads again on every
     * loop, so a number edited in Panels takes effect right away, even while an OpMode runs.
     * Panels edits are NOT saved to this file and are lost when the Robot Controller app
     * restarts: when a value is right, copy it here. Hardware names, the encoder port and the
     * physical constants ({@link #GEAR_RATIO}, {@link #SERVO_FREE_SPEED_RPM}, ...) stay
     * {@code final}. Change a direction flag (ENCODER_REVERSED, SERVO_*_REVERSED) only while
     * the turret is stopped.
     */
    @Configurable
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
         * The drive motors and the intake motor qualify (they run without their encoders).
         * Do NOT use the transfer or outtake ports (they run on their encoders).
         * REV recommends hub motor ports 0 or 3 for this high-resolution encoder.
         *
         * <p>This name MUST be the motor whose encoder socket the cable is really in. If it
         * is not, the measured turret angle never changes and the turret cannot aim. The
         * turret code only reads the count; it never changes that motor's mode, power or
         * direction.
         */
        // Changed from Intake.MOTOR on 2026-11-20. The turret spun slow circles in TRACKING
        // because the count read through the intake port never changed, so the measured
        // angle was stuck at 0. Branch today_working-one had already moved this to the
        // front-right drive port on 2026-11-16 (commit 4742a74); this branch never got that
        // change. CONFIRM IT on the Main TeleOp init screen: turn the turret by hand, the
        // screen lists every motor port whose encoder count is changing. The port it lists
        // is the name that belongs here. (Pedro does not use the drive motor encoders; the
        // robot is localized by the Pinpoint.)
        public static final String ENCODER_PORT = Drive.FRONT_RIGHT;

        /** Quadrature counts per revolution of the REV Through Bore Encoder V2 (2048 CPR x 4). */
        public static final double ENCODER_TICKS_PER_REV = 8192.0;

        /** Servo turns per one turret turn, fixed by the 2.75:1 gear-driven turret. */
        public static final double GEAR_RATIO = 2.75;

        /**
         * Turret turns per ONE encoder turn. {@code 1 / GEAR_RATIO} is only right if the
         * encoder turns exactly as often as a servo pinion (it rides on a servo shaft, or on
         * its own gear with the SAME tooth count as the servo pinion). With the encoder on
         * its own gear on the turret ring, this is
         * {@code encoder gear teeth / turret ring teeth}.
         *
         * <p>Measure it instead of trusting it: on the Main TeleOp init screen, zero the
         * turret (Gamepad 2 Y), turn it exactly ONE full turn by hand and read
         * {@code ticks since zero}. Then set this to {@code 8192.0 / that number}
         * (22528 ticks means the value below is already right). A wrong value does not make
         * the turret spin; it makes it aim off by a constant factor.
         */
        // TODO(team): measure on the robot (see above)
        public static double ENCODER_TO_TURRET_RATIO = 1.0 / GEAR_RATIO;

        /**
         * Set true if turning the turret counterclockwise (looking down from above, the
         * +angle direction) makes {@code Turret measuredAngleDeg} go DOWN. A pinion on a
         * ring gear spins opposite to the ring, so this often needs to be true.
         *
         * <p>Check it in INIT of Main TeleOp, before Play: the turret servos are unpowered
         * there and the init screen shows the live angle and ticks, so turn the turret by
         * hand and watch. The sign also depends on the motor TYPE chosen for the shared
         * port in the Robot Controller config (some types, such as goBILDA 5202 and REV HD
         * Hex, are counted in reverse), so re-check this flag after changing the type or
         * {@link #ENCODER_PORT}. The shared motor's own FORWARD/REVERSE setting does not
         * matter: TurretSubsystem cancels it out.
         */
        // Set true on 2026-11-18 as a guess (the encoder was not being read that day, see
        // ENCODER_PORT). NOT YET VERIFIED. Verify on the init screen: turret turned LEFT by
        // hand -> measuredAngleDeg must go UP. If it goes DOWN, set this to false.
        // TODO(team): verify on the robot
        public static boolean ENCODER_REVERSED = true;

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
        // Set true on 2026-11-18 together with ENCODER_REVERSED (see above). NOTE: on that
        // day the encoder was not being read (see ENCODER_PORT), so ENCODER_REVERSED was a
        // guess and still has to be checked by hand on the init screen. If the servo and
        // encoder directions disagree the turret cannot run away: TurretSubsystem notices
        // the encoder moving against the commanded power, flips the servo direction at run
        // time and shows "SERVO DIRECTION AUTO-FLIPPED" on the Driver Station with the
        // steps to find out which flag is the wrong one.
        public static boolean SERVO_LEFT_REVERSED = true;
        public static boolean SERVO_RIGHT_REVERSED = true;

        /**
         * Servo power used while the operator jogs the turret by hand with Gamepad 2
         * Dpad Left / Right. Also the quickest direction check: Dpad LEFT must turn the
         * turret LEFT (counterclockwise from above) and make measuredAngleDeg go UP.
         */
        public static double JOG_POWER = 0.34;

        /**
         * Direction monitor: a commanded power at or above this (either sign) is expected
         * to move the turret THAT way. If instead the encoder moves the other way by
         * {@link #RUNAWAY_DEGREES}, the servo and encoder directions disagree and the
         * subsystem flips the servo direction at run time (see {@link #SERVO_LEFT_REVERSED}).
         */
        public static double DIRECTION_MONITOR_MIN_POWER = 0.30;

        /** See {@link #DIRECTION_MONITOR_MIN_POWER}. Big enough that a hand bump cannot trigger it. */
        public static double RUNAWAY_DEGREES = 10.0;

        /**
         * Coast allowance for the direction monitor, in seconds. After a fast move the
         * turret coasts past the target while the command has already reversed. That is
         * normal, not a reversed servo, so on top of {@link #RUNAWAY_DEGREES} the turret may
         * travel against a new command by (its speed when the command reversed) x (this
         * time) before the monitor reacts. Without it, a heavy turret's overshoot causes a
         * false flip, and a false flip makes the turret run away for real. Raise this if
         * "SERVO DIRECTION AUTO-FLIPPED" shows up on a turret whose directions are right.
         */
        public static double DIRECTION_MONITOR_COAST_SECONDS = 0.3;

        /**
         * Direction monitor, second test. With reversed directions and the target behind
         * the turret, the turret does not run away: it gets pushed to the point exactly
         * OPPOSITE the target and buzzes there, the command reversing every loop or two.
         * A correct turret never reverses its command while it is more than 90 degrees
         * from the target (it only does that once, when the target passes directly
         * behind). This many reversals in {@link #ANTIPODE_CHATTER_SECONDS}, all more than
         * 90 degrees from the target, are treated as reversed directions.
         */
        public static int ANTIPODE_CHATTER_REVERSALS = 4;

        /** See {@link #ANTIPODE_CHATTER_REVERSALS}. */
        public static double ANTIPODE_CHATTER_SECONDS = 1.0;

        /**
         * Most run-time servo direction flips allowed in one OpMode. One flip fixes a
         * wrong flag. If the monitor wants more than this, something else is wrong (the
         * encoder is slipping or reading another mechanism), so the turret is stopped
         * instead of being flipped back and forth.
         */
        public static int MAX_AUTO_FLIPS = 2;

        /**
         * Encoder monitor. While HOME or TRACKING is driving the servos with any power
         * meant to move the turret ({@link #MIN_COMMAND_POWER} or more), the encoder count
         * must change. If it moves by less than {@link #ENCODER_STALL_MIN_TICKS} for
         * {@link #ENCODER_STALL_SECONDS} (at or above this power) or for
         * {@link #ENCODER_STALL_SLOW_SECONDS} (below this power), the turret is not being
         * measured (encoder cable out, wrong port) or cannot move. The servos are then
         * stopped instead of spinning open-loop, and the Driver Station shows
         * "TURRET STOPPED: ENCODER NOT COUNTING". Manual jog (Dpad Left / Right) still
         * works; Gamepad 2 Dpad Down or Up clears the stop and retries.
         */
        public static double ENCODER_STALL_MIN_POWER = 0.30;

        /** See {@link #ENCODER_STALL_MIN_POWER}. */
        public static double ENCODER_STALL_SECONDS = 0.5;

        /**
         * See {@link #ENCODER_STALL_MIN_POWER}. Longer than the fast time so a turret that
         * is slow to break away at the minimum power is not stopped by mistake. If a turret
         * with a WORKING encoder gets stopped a degree or two short of the target, the
         * minimum power is too weak to move it: measure {@link #SERVO_DEADBAND_POWER}.
         */
        public static double ENCODER_STALL_SLOW_SECONDS = 1.0;

        /**
         * See {@link #ENCODER_STALL_MIN_POWER}. About half a degree of turret travel. More
         * than one count, so an encoder with one dead channel (the count flickers by 1)
         * still counts as not counting.
         */
        public static int ENCODER_STALL_MIN_TICKS = 30;

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
        public static double RAD_PER_SEC_AT_FULL_POWER =
                SERVO_FREE_SPEED_RPM * 2.0 * Math.PI / 60.0 / GEAR_RATIO;

        /**
         * Servo power at or below which the turret does NOT move (servo dead zone plus
         * turret friction). Used to size {@link #MIN_COMMAND_POWER} and {@link #KF}.
         *
         * <p>To measure: command the turret servos at a fixed low power and raise it in
         * small steps until the turret just starts to turn in both directions; use the
         * larger of the two values.
         *
         * <p>This one stays a code-only number: {@link #MIN_COMMAND_POWER}, {@link #KF} and
         * {@link #KV} START from it but are separate Panels numbers, so after measuring it
         * edit those three too.
         */
        // TODO(team): measure on the robot (0.08 is only a placeholder)
        public static final double SERVO_DEADBAND_POWER = 0.08;

        /**
         * Smallest power used for a correction TOWARD the target while the turret is outside
         * {@link #ANGLE_TOLERANCE_DEGREES}. Must stay above {@link #SERVO_DEADBAND_POWER} so
         * the correction actually moves the turret. While the target is moving, weaker (or
         * zero) power can still be sent: feedforward in the other direction is left alone,
         * and a turret that is ahead of a target coming toward it just waits. Raising this
         * makes the turret step in bigger jumps and it can start to hunt back and forth.
         */
        public static double MIN_COMMAND_POWER = SERVO_DEADBAND_POWER + 0.04;

        /**
         * Time constant (seconds) of the low-pass filter on the target's angular velocity,
         * which feeds {@link #KV}. Larger is smoother but lags more; 0 turns the filter off.
         */
        public static double TARGET_VELOCITY_FILTER_SECONDS = 0.10;

        /**
         * Target angular speed (degrees/second) above which the target counts as MOVING.
         * While it is moving, the turret keeps following with feedforward even inside
         * {@link #ANGLE_TOLERANCE_DEGREES} instead of stopping dead, so it does not
         * stop-start while the chassis spins. Below this speed the turret stops once aimed.
         */
        public static double TARGET_MOVING_THRESHOLD_DEG_SEC = 10.0;

        /**
         * Longest time (milliseconds) the TeleOp shoot sequence waits for the turret to
         * finish aiming before it fires anyway. Only applies while the turret is TRACKING.
         */
        public static double AIM_WAIT_TIMEOUT_MS = 750.0;

        /**
         * Turret pivot offset from the robot's localization center in inches
         * (+X is robot forward, +Y is robot left).
         */
        // TODO(team): measure turret pivot offset from robot center in inches
        public static double OFFSET_X_INCHES = 0.0;
        // 2.0 taken from branch Shot-Feeder-Test (commit 31f0e2c, 2026-11-17). +Y is the
        // robot's LEFT; if the turret actually sits 2 in to the right, use -2.0.
        public static double OFFSET_Y_INCHES = 2.0;

        /**
         * The two hive sides in Pedro Pathing inches. The turret aims at the TOP side when the
         * robot is above SIDE_SWITCH_Y_INCHES and at the BOTTOM side when it is below.
         *
         * <p>Frame: the same one as {@link Field#START_POSE} and the autonomous paths, with
         * the origin at a field corner and both axes running 0 to 144 inches (NOT the
         * center-origin -72 to 72 frame). Every turret target must be given in this frame.
         */
        // TODO(team): replace these placeholders with the real hive coordinates (read them in the Pedro visualizer)
        public static double TOP_TARGET_X_INCHES = 60;
        public static double TOP_TARGET_Y_INCHES = 94;
        public static double BOTTOM_TARGET_X_INCHES = 52;
        public static double BOTTOM_TARGET_Y_INCHES = 53;

        /** Y position (inches) where the aimed side switches; 72 is the middle of a 144-inch field. */
        public static double SIDE_SWITCH_Y_INCHES = 72.0;

        /** Dead band (inches) around the switch line so the target does not flicker. */
        public static double SIDE_HYSTERESIS_INCHES = 3.0;

        /**
         * Chassis-relative angle (degrees) the turret turns to and holds in HOME mode
         * (Gamepad 2 Dpad Up). 0 is straight ahead on the robot, + is counterclockwise (left).
         */
        public static double HOME_ANGLE_DEGREES = 0.0;

        /**
         * Whether soft rotation limits are enforced to prevent wire binding. {@code false}
         * means the turret spins without limit and always takes the shortest path to the
         * target. Set back to {@code true} if any cable can wrap around the turret.
         */
        public static boolean USE_SOFT_LIMITS = false;

        /**
         * Allowed turret angles relative to the chassis (0 = forward, + = counterclockwise).
         * Only used while {@link #USE_SOFT_LIMITS} is {@code true}.
         */
        // TODO(team): measure safe mechanical rotation limits in degrees
        public static double MIN_ANGLE_DEGREES = -180.0;
        public static double MAX_ANGLE_DEGREES = 180.0;

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
        public static double MAX_POWER = 1;

        /**
         * TURNING SPEED PROFILE, knob 1 of 2: how far from the target (degrees) the turret
         * STARTS slowing down.
         *
         * <p>Farther from the target than this, the turret runs flat out at
         * {@link #MAX_POWER}. Once it is closer than this, the power is reduced by
         * {@link #SLOWDOWN_POWER_PER_DEGREE} for every degree it gets closer, but never
         * below {@link #MIN_COMMAND_POWER}. Inside {@link #ANGLE_TOLERANCE_DEGREES} it stops.
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
        public static double SLOWDOWN_START_DEGREES = 70.0;

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
        public static double SLOWDOWN_POWER_PER_DEGREE = 0.012;

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
        public static double KI = 0.0;
        public static double KD = 0.0;
        public static double KV = (1.0 - SERVO_DEADBAND_POWER) / RAD_PER_SEC_AT_FULL_POWER;
        public static double KF = SERVO_DEADBAND_POWER;

        /** Maximum integral accumulation (radians * seconds). */
        public static double MAX_INTEGRAL = 0.5;

        /** Angle error deadband (degrees) within which the turret counts as aimed. */
        public static double ANGLE_TOLERANCE_DEGREES = .75;

        /**
         * Degrees. Once aimed on a stationary target, the turret stays still until the error
         * exceeds this. Must be larger than ANGLE_TOLERANCE_DEGREES. It has to be wider than
         * ONE correction kick: the degrees the turret moves in one loop at MIN_COMMAND_POWER,
         * plus about 1 degree.
         */
        public static double ANGLE_EXIT_TOLERANCE_DEGREES = 1.5;
    }

    /**
     * Gamepad 1 Y in Main TeleOp: the chassis turns to face the hive (see
     * {@link ChassisAligner}). It uses the hive coordinates, side switch and pivot offset in
     * {@link Turret}.
     *
     * <p>Tunable from Panels (RobotConstants > ChassisAlign). Panels edits are NOT saved
     * to this file and are lost when the Robot Controller app restarts: when a value is
     * right, copy it here.
     */
    @Configurable
    public static final class ChassisAlign {
        /**
         * True holds the turret straight ahead while aligning, so the chassis does all the
         * aiming. False leaves the turret doing whatever it was doing.
         */
        public static boolean HOLD_TURRET_STRAIGHT = true;

        // TODO(team): tune the values below on the robot
        /** Turn power per radian of heading error. Raise if slow, lower if it overshoots. */
        public static double KP = 1.6;
        /** Turn power per radian/second of error change. Damps overshoot. */
        public static double KD = 0.12;
        /** Turn power added toward the target to overcome friction. Raise if it stops short. */
        public static double KS = 0.08;
        /** Largest turn power used while aligning, in [0, 1]. */
        public static double MAX_TURN_POWER = 0.6;
        /** Heading error (degrees) at which the robot counts as aligned and stops turning. */
        public static double TOLERANCE_DEGREES = 1.0;
        /** Once aligned, it stays still until the error exceeds this (degrees). */
        public static double EXIT_TOLERANCE_DEGREES = 2.0;
        /** Constant correction to the aim, degrees. Positive turns the robot further left. */
        public static double TRIM_DEGREES = 0.0;
    }

    /**
     * Two-position flower ramp servo.
     *
     * <p>Servo positions are unitless in {@code [0.0, 1.0]}. 0.0 and 1.0
     * are the mechanical ends of travel; measure the two poses you actually
     * want and put them here.
     *
     * <p>Tunable from Panels (RobotConstants > FlowerRamp). Panels edits are NOT saved
     * to this file and are lost when the Robot Controller app restarts: when a position
     * is right, copy the number here.
     */
    @Configurable
    public static final class FlowerRamp {
        public static final String NAME = "flower";

        /**
         * First pose. Used at init so the mechanism starts in a known place.
         */
        // TODO(team): measure this on the robot
        public static double POSITION_A = 1;

        /**
         * Second pose.
         */
        // TODO(team): measure this on the robot
        public static double POSITION_B = .46;

        /**
         * Third pose. {@link TestAuto} twitches between this and {@link #POSITION_B}
         * while it waits at point 5.
         */
        // TODO(team): measure this on the robot
        public static double POSITION_C = .47;

        /** Time the twitch holds each of the two poses before switching, milliseconds. */
        // TODO(team): tune on the robot
        public static double TWITCH_INTERVAL_MS = 500;
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
        public static final double GATE_OPEN_WAIT_MS = 800.0;

        /** How long the intake and transfer run to feed every ball into the flywheels. */
        // TODO(team): tune on the robot (long enough for all balls to leave)
        public static final double FEED_TIME_MS = 700.0;
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
        public static final Pose OPPOSE_POSE = new Pose(58, 135, Math.toRadians(270.0));
        /** Shooting spot in the bottom half; Main TeleOp drives here with Gamepad 1 X. */
        public static final Pose BOTTOM_SHOOT = new Pose(58, 14, Math.toRadians(90.0));
        public static final Pose BOTTOM_ANGLE_SHOOT = new Pose(25.4, 17.3, Math.toRadians(55));
        public static final Pose TOP_ANGLE_SHOOT = new Pose(27, 140, Math.toRadians(325));
        public static final Pose PICKUP_POSE = new Pose(24.0, 0.0, Math.toRadians(0.0));
        public static final Pose SCORE_POSE = new Pose(24.0, 24.0, Math.toRadians(90.0));
        public static final Pose PARK_POSE = new Pose(48.0, 24.0, Math.toRadians(90.0));
    }

    /**
     * Path points for {@link TestAuto}, editable from Panels.
     *
     * <p>Units: X and Y are inches in the same Pedro Pathing field frame as the rest of the
     * code (origin at a field corner, both axes 0 to 144). HEADING is degrees (0 = facing
     * +X, counterclockwise positive). A control point only has X and Y: it shapes a Bezier
     * curve and has no heading.
     *
     * <p>{@link TestAuto} reads these while it waits for Play (every INIT loop) and once
     * more when Play is pressed, and builds all of its paths at that moment. So edit them
     * before pressing Play; a value changed after Play does not affect a run that is already
     * going. Editing the START values while waiting also moves the follower to the new start
     * pose.
     *
     * <p>Panels edits are NOT saved to this file and are lost when the Robot Controller app
     * restarts: when a point is right, copy the number here.
     *
     * <p>Route order: start -> point1 -> point2 (curve via point2Control1) -> point3 (curve
     * via point3Control1 and point3Control2) -> point4 (curve via point4Control1 and
     * point4Control2) -> point5 -> back to point4 -> point3 (the point 4 curve driven
     * backwards) -> point6 -> park.
     */
    @Configurable
    public static final class TestAutoPoints {

        // @Sorter sets the order Panels lists these in (lowest first). Give every new value
        // one too, or it is listed at the bottom.

        // ---- Start pose (the pose the follower and the turret start from) ----
        @Sorter(sort = 0) public static double START_X = 56;
        @Sorter(sort = 1) public static double START_Y = 8;
        @Sorter(sort = 2) public static double START_HEADING_DEG = 90;

        // ---- Point 1: straight line from start ----
        @Sorter(sort = 10) public static double POINT1_X = 35;
        @Sorter(sort = 11) public static double POINT1_Y = 23;
        @Sorter(sort = 12) public static double POINT1_HEADING_DEG = 180;

        // ---- Point 2: curve from point 1, heading held at the point 2 heading ----
        @Sorter(sort = 20) public static double POINT2_X = 16;
        @Sorter(sort = 21) public static double POINT2_Y = 14;
        @Sorter(sort = 22) public static double POINT2_HEADING_DEG = 180;
        /** Bezier control point of the point 1 -> point 2 curve. */
        @Sorter(sort = 23) public static double POINT2_CONTROL1_X = 80;
        @Sorter(sort = 24) public static double POINT2_CONTROL1_Y = 20;
        /**
         * Speed limit for the point 1 -> point 2 curve only, as a fraction of the robot's
         * top speed: 0.3 is 70% slower, 1 is no limit. Kept between 0.1 and 1.
         */
        @Sorter(sort = 25) public static double POINT2_SPEED = 0.7;

        // ---- Point 3: curve from point 2 (a shot is taken here) ----
        @Sorter(sort = 30) public static double POINT3_X = 59;
        @Sorter(sort = 31) public static double POINT3_Y = 146;
        @Sorter(sort = 32) public static double POINT3_HEADING_DEG = 270;
        /** First Bezier control point of the point 2 -> point 3 curve. */
        @Sorter(sort = 33) public static double POINT3_CONTROL1_X = 37;
        @Sorter(sort = 34) public static double POINT3_CONTROL1_Y = 122;
        /** Second Bezier control point of the point 2 -> point 3 curve. */
        @Sorter(sort = 35) public static double POINT3_CONTROL2_X = 35;
        @Sorter(sort = 36) public static double POINT3_CONTROL2_Y = 115;

        // ---- Point 4: curve from point 3; also the start of the point 4 -> point 3 return ----
        @Sorter(sort = 40) public static double POINT4_X = 45;
        @Sorter(sort = 41) public static double POINT4_Y = 57;
        @Sorter(sort = 42) public static double POINT4_HEADING_DEG = 180;
        /** First Bezier control point of the point 3 -> point 4 curve (last on the way back). */
        @Sorter(sort = 43) public static double POINT4_CONTROL1_X = 43;
        @Sorter(sort = 44) public static double POINT4_CONTROL1_Y = 115;
        /** Second Bezier control point of the point 3 -> point 4 curve (first on the way back). */
        @Sorter(sort = 45) public static double POINT4_CONTROL2_X = 31;
        @Sorter(sort = 46) public static double POINT4_CONTROL2_Y = 122;

        // ---- Point 5: straight line from point 4 and back ----
        @Sorter(sort = 50) public static double POINT5_X = 21;
        @Sorter(sort = 51) public static double POINT5_Y = 57;
        @Sorter(sort = 52) public static double POINT5_HEADING_DEG = 180;

        // ---- Point 6: straight line from point 3 ----
        @Sorter(sort = 60) public static double POINT6_X = 56;
        @Sorter(sort = 61) public static double POINT6_Y = 8;
        @Sorter(sort = 62) public static double POINT6_HEADING_DEG = 90;

        // ---- Park: final straight line from point 6 ----
        @Sorter(sort = 70) public static double PARK_X = 17;
        @Sorter(sort = 71) public static double PARK_Y = 93;
        @Sorter(sort = 72) public static double PARK_HEADING_DEG = 90;
    }
}