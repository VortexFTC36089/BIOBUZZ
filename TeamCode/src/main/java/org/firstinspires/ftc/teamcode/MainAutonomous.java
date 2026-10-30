package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoPoses.Alliance;
import org.firstinspires.ftc.teamcode.AutoPoses.Routine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Match autonomous built with Pedro Pathing 3 and Ivy.
 *
 * <ol>
 *   <li>Start at shooting position 1, spin up, shoot the preloaded balls.</li>
 *   <li>Drive to the lower flower, creep forward with the intake running
 *       to collect, creep back.</li>
 *   <li>Drive to shooting position 2 at the hive (flywheel spinning up on
 *       the way) and shoot.</li>
 *   <li>Drive to the second flower, creep forward with the intake running,
 *       creep back.</li>
 *   <li>Return to shooting position 2 (flywheel spinning up on the way)
 *       and shoot.</li>
 * </ol>
 *
 * <p>Pick the alliance during init (X = blue, B = red). Poses come from
 * {@link AutoPoses}; nothing in this file is a field coordinate.
 *
 * <p>Hardware: mecanum drive and Pinpoint from {@link Constants}; the
 * intake motor, shooter motor and feed servo from {@link RobotConstants}.
 */
@Autonomous(name = "Main Autonomous", group = RobotConstants.OpModeGroups.AUTO, preselectTeleOp = "Main TeleOp")
public class MainAutonomous extends OpMode {

    /**
     * Timing limits. Path timeouts are safety nets, not targets: a well
     * tuned robot finishes long before them.
     */
    private static final class Timing {
        /** Give up on a long drive after this many milliseconds. */
        static final int PATH_TIMEOUT_MS = 5000;
        /** Give up on the short flower creep after this many milliseconds. */
        static final int CREEP_TIMEOUT_MS = 2000;
        /** Keep intaking this long after the creep-back finishes so the
         *  last ball is fully inside before driving off. */
        // TODO(team): tune on the robot
        static final int INTAKE_SETTLE_MS = 500;
    }

    /**
     * Fraction of full path speed while creeping into a flower, so the
     * intake has time to grab balls. 1.0 is full speed.
     */
    // TODO(team): tune on the robot
    private static final double FLOWER_CREEP_SPEED = 0.4;

    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private Servo feedServo;

    private Alliance alliance = Alliance.BLUE;
    private Routine routine;

    private Path toFlower1;
    private Path flower1Forward;
    private Path flower1Back;
    private Path flower1ToShootPosition2;
    private Path toFlower2;
    private Path flower2Forward;
    private Path flower2Back;
    private Path flower2ToShootPosition2;

    private String currentStep = "INIT";
    private final ElapsedTime matchTimer = new ElapsedTime();

    /**
     * Builds hardware and closes the feed so preloads stay put.
     */
    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        feedServo = hardwareMap.get(Servo.class, RobotConstants.Servo.NAME);
        closeFeed();

        // Hold position at the end of each path so the robot does not drift
        // while shooting or intaking.
        follower.holdEnd.set(true);

        applyAlliance(Alliance.BLUE);

        telemetry.addLine("Main Autonomous (Ivy) ready");
        telemetry.addLine("X = BLUE   B = RED   (pick before Play)");
        telemetry.update();
    }

    /**
     * Lets the drive team pick the alliance until Play is pressed.
     */
    @Override
    public void init_loop() {
        if (gamepad1.x) {
            applyAlliance(Alliance.BLUE);
        } else if (gamepad1.b) {
            applyAlliance(Alliance.RED);
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start pose", routine.shootPosition1);
        telemetry.addLine("X = BLUE   B = RED");
        telemetry.update();
    }

    /**
     * Locks in the start pose and schedules the Ivy autonomous routine.
     */
    @Override
    public void start() {
        Scheduler.reset();
        follower.setPose(routine.shootPosition1);
        follower.update();
        matchTimer.reset();
        schedule(autoRoutine());
    }

    /**
     * Executes the Ivy scheduler and updates hardware each loop.
     */
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        intake.update();
        shooter.update();
        PoseStorage.lastAutoPose = follower.pose();

        writeTelemetry();
    }

    /**
     * Stops everything and records the final pose for TeleOp.
     */
    @Override
    public void stop() {
        Scheduler.reset();
        if (intake != null) {
            intake.stop();
            intake.update();
        }
        if (shooter != null) {
            shooter.stop();
            shooter.update();
        }
        if (follower != null) {
            PoseStorage.lastAutoPose = follower.pose();
            follower.stop();
        }
    }

    // ---- Ivy Command Helpers ----

    /**
     * Starts the intake motor.
     */
    private Command runIntake() {
        return instant(intake::intake);
    }

    /**
     * Stops the intake motor.
     */
    private Command stopIntake() {
        return instant(intake::stop);
    }

    /**
     * Follows a path with a safety timeout so an untuned robot does not
     * stall for the rest of autonomous.
     */
    private Command followWithTimeout(Path path, int timeoutMs) {
        return race(
                follow(follower, path),
                waitMs(timeoutMs)
        );
    }

    /**
     * Spins up the shooter, waits until it reaches target speed (or spin-up
     * timeout), opens the feed servo + runs the intake for {@code SHOOT_SECONDS},
     * then closes the feed and stops the intake and shooter.
     */
    private Command shootCommand(String stepName) {
        int shootMs = (int) (RobotConstants.Shooter.SHOOT_SECONDS * 1000.0);
        return sequential(
                instant(() -> {
                    currentStep = stepName + " (SPIN UP)";
                    shooter.spinUp();
                }),
                waitUntil(shooter::isReady),
                instant(() -> {
                    currentStep = stepName;
                    openFeed();
                }),
                runIntake(),
                waitMs(shootMs),
                instant(this::closeFeed),
                stopIntake(),
                instant(shooter::stop)
        );
    }

    /**
     * Collects balls from a flower: creeps forward while intaking, then
     * creeps back while waiting at least {@code INTAKE_SETTLE_MS} so the
     * last ball settles inside before driving off.
     */
    private Command collectFlower(String stepName, Path creepForward, Path creepBack) {
        return sequential(
                instant(() -> currentStep = stepName + " FORWARD"),
                runIntake(),
                followWithTimeout(creepForward, Timing.CREEP_TIMEOUT_MS),
                instant(() -> currentStep = stepName + " BACK"),
                parallel(
                        followWithTimeout(creepBack, Timing.CREEP_TIMEOUT_MS),
                        waitMs(Timing.INTAKE_SETTLE_MS)
                ),
                stopIntake()
        );
    }

    /**
     * Full match autonomous sequence composed with Ivy.
     */
    private Command autoRoutine() {
        return sequential(
                // 1. Shoot preloaded balls from start position
                shootCommand("SHOOT_1"),

                // 2. Drive to lower flower and collect
                instant(() -> currentStep = "DRIVE_TO_FLOWER_1"),
                followWithTimeout(toFlower1, Timing.PATH_TIMEOUT_MS),
                collectFlower("FLOWER_1", flower1Forward, flower1Back),

                // 3. Drive to shooting position 2 while spinning up, then shoot
                instant(() -> {
                    currentStep = "DRIVE_TO_SHOOT_POSITION_2";
                    shooter.spinUp();
                }),
                followWithTimeout(flower1ToShootPosition2, Timing.PATH_TIMEOUT_MS),
                shootCommand("SHOOT_2"),

                // 4. Drive to second flower and collect
                instant(() -> currentStep = "DRIVE_TO_FLOWER_2"),
                followWithTimeout(toFlower2, Timing.PATH_TIMEOUT_MS),
                collectFlower("FLOWER_2", flower2Forward, flower2Back),

                // 5. Return to shooting position 2 while spinning up, then shoot
                instant(() -> {
                    currentStep = "RETURN_TO_SHOOT_POSITION_2";
                    shooter.spinUp();
                }),
                followWithTimeout(flower2ToShootPosition2, Timing.PATH_TIMEOUT_MS),
                shootCommand("SHOOT_3"),

                instant(() -> currentStep = "DONE")
        );
    }

    /**
     * Rebuilds poses and paths for the chosen alliance.
     *
     * <p>Paths are built here, not in {@link #start()}, so a bad pose shows
     * up as an init error instead of a crash when the match begins.
     *
     * @param chosen alliance selected on the gamepad
     */
    private void applyAlliance(Alliance chosen) {
        alliance = chosen;
        routine = AutoPoses.forAlliance(chosen);

        Pose flower1In = AutoPoses.forwardOf(routine.flower1, AutoPoses.FLOWER_PUSH_INCHES);
        Pose flower2In = AutoPoses.forwardOf(routine.flower2, AutoPoses.FLOWER_PUSH_INCHES);

        // line(a, b) is the straight segment; .linear(a, b) turns the
        // heading smoothly from a's heading to b's along the way;
        // .constant(p) keeps the heading fixed for the whole segment.
        toFlower1 = line(routine.shootPosition1, routine.flower1)
                .linear(routine.shootPosition1, routine.flower1);

        flower1Forward = creep(routine.flower1, flower1In);
        flower1Back = creep(flower1In, routine.flower1);

        flower1ToShootPosition2 = line(routine.flower1, routine.shootPosition2)
                .linear(routine.flower1, routine.shootPosition2);

        toFlower2 = line(routine.shootPosition2, routine.flower2)
                .linear(routine.shootPosition2, routine.flower2);

        flower2Forward = creep(routine.flower2, flower2In);
        flower2Back = creep(flower2In, routine.flower2);

        flower2ToShootPosition2 = line(routine.flower2, routine.shootPosition2)
                .linear(routine.flower2, routine.shootPosition2);
    }

    /**
     * Short, slow, fixed-heading segment used to nudge into and out of a
     * flower.
     *
     * @param from start pose
     * @param to   end pose (same heading as {@code from})
     * @return the path
     */
    private Path creep(Pose from, Pose to) {
        return line(from, to)
                .constant(from)
                .with(Constants.foresightConfig.maxPathSpeed.at(FLOWER_CREEP_SPEED));
    }

    private void openFeed() {
        feedServo.setPosition(RobotConstants.Servo.POSITION_B);
    }

    private void closeFeed() {
        feedServo.setPosition(RobotConstants.Servo.POSITION_A);
    }

    /**
     * One telemetry.update() per loop.
     */
    private void writeTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Step", currentStep);
        telemetry.addData("Match time (s)", "%.1f", matchTimer.seconds());
        telemetry.addData("X (in)", "%.1f", pose.x());
        telemetry.addData("Y (in)", "%.1f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Follower", follower.mode());
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Shooter", shooter.isReady() ? "READY" : (shooter.isSpinning() ? "SPINNING UP" : "OFF"));
        telemetry.update();
    }
}
