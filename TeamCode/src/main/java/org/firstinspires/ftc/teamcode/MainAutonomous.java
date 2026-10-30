package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
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

/**
 * Match autonomous.
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
 * <p>The routine is a state machine run from {@link #loop()} rather than a
 * {@code LinearOpMode} so mechanisms can run while a path is being followed
 * and so every state has a timeout. An untuned robot that never reaches a
 * pose moves on instead of sitting still for 30 seconds.
 *
 * <p>Hardware: mecanum drive and Pinpoint from {@link Constants}; the
 * intake motor, shooter motor and feed servo from {@link RobotConstants}.
 */
@Autonomous(name = "Main Autonomous", group = RobotConstants.OpModeGroups.AUTO, preselectTeleOp = "Main TeleOp")
public class MainAutonomous extends OpMode {

    /**
     * Steps of the routine, in order.
     */
    private enum State {
        SPIN_UP_1,
        SHOOT_1,
        DRIVE_TO_FLOWER_1,
        INTAKE_FLOWER_1_FORWARD,
        INTAKE_FLOWER_1_BACK,
        DRIVE_TO_SHOOT_POSITION_2,
        SHOOT_2,
        DRIVE_TO_FLOWER_2,
        INTAKE_FLOWER_2_FORWARD,
        INTAKE_FLOWER_2_BACK,
        RETURN_TO_SHOOT_POSITION_2,
        SHOOT_3,
        DONE
    }

    /**
     * Timing limits. Path timeouts are safety nets, not targets: a well
     * tuned robot finishes long before them.
     */
    private static final class Timing {
        /** Give up on a long drive after this many seconds. */
        static final double PATH_TIMEOUT_SECONDS = 5.0;
        /** Give up on the short flower creep after this many seconds. */
        static final double CREEP_TIMEOUT_SECONDS = 2.0;
        /** Keep intaking this long after the creep-back finishes so the
         *  last ball is fully inside before driving off. */
        // TODO(team): tune on the robot
        static final double INTAKE_SETTLE_SECONDS = 0.5;
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

    private State state = State.SPIN_UP_1;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    /**
     * Builds hardware and closes the feed so preloads stay put.
     */
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        feedServo = hardwareMap.get(Servo.class, RobotConstants.Servo.NAME);
        closeFeed();

        // Hold position at the end of each path so the robot does not drift
        // while shooting or intaking.
        follower.holdEnd.set(true);

        applyAlliance(Alliance.BLUE);

        telemetry.addLine("Main Autonomous ready");
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
     * Locks in the start pose and starts the flywheel.
     */
    @Override
    public void start() {
        follower.setPose(routine.shootPosition1);
        matchTimer.reset();
        enter(State.SPIN_UP_1);
    }

    /**
     * Runs one step of the state machine, then writes to hardware.
     */
    @Override
    public void loop() {
        switch (state) {
            // ---- Shot 1: preloads from the start position ----
            case SPIN_UP_1:
                if (shooter.isReady()) {
                    enter(State.SHOOT_1);
                }
                break;

            case SHOOT_1:
                if (shotFinished()) {
                    enter(State.DRIVE_TO_FLOWER_1);
                }
                break;

            // ---- Collect from the lower flower ----
            case DRIVE_TO_FLOWER_1:
                if (pathFinished(Timing.PATH_TIMEOUT_SECONDS)) {
                    enter(State.INTAKE_FLOWER_1_FORWARD);
                }
                break;

            case INTAKE_FLOWER_1_FORWARD:
                if (pathFinished(Timing.CREEP_TIMEOUT_SECONDS)) {
                    enter(State.INTAKE_FLOWER_1_BACK);
                }
                break;

            case INTAKE_FLOWER_1_BACK:
                if (creepBackFinished()) {
                    enter(State.DRIVE_TO_SHOOT_POSITION_2);
                }
                break;

            // ---- Shot 2: at the hive from shooting position 2 ----
            case DRIVE_TO_SHOOT_POSITION_2:
                if (pathFinished(Timing.PATH_TIMEOUT_SECONDS) && shooter.isReady()) {
                    enter(State.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (shotFinished()) {
                    enter(State.DRIVE_TO_FLOWER_2);
                }
                break;

            // ---- Collect from the second flower ----
            case DRIVE_TO_FLOWER_2:
                if (pathFinished(Timing.PATH_TIMEOUT_SECONDS)) {
                    enter(State.INTAKE_FLOWER_2_FORWARD);
                }
                break;

            case INTAKE_FLOWER_2_FORWARD:
                if (pathFinished(Timing.CREEP_TIMEOUT_SECONDS)) {
                    enter(State.INTAKE_FLOWER_2_BACK);
                }
                break;

            case INTAKE_FLOWER_2_BACK:
                if (creepBackFinished()) {
                    enter(State.RETURN_TO_SHOOT_POSITION_2);
                }
                break;

            // ---- Shot 3: back at shooting position 2 ----
            case RETURN_TO_SHOOT_POSITION_2:
                if (pathFinished(Timing.PATH_TIMEOUT_SECONDS) && shooter.isReady()) {
                    enter(State.SHOOT_3);
                }
                break;

            case SHOOT_3:
                if (shotFinished()) {
                    enter(State.DONE);
                }
                break;

            case DONE:
            default:
                // Nothing left to do; the follower holds the last pose.
                break;
        }

        intake.update();
        shooter.update();
        follower.update();
        PoseStorage.lastAutoPose = follower.pose();

        writeTelemetry();
    }

    /**
     * Stops everything and records the final pose for TeleOp.
     */
    @Override
    public void stop() {
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

    /**
     * Switches state, restarts the state timer, and kicks off whatever
     * that state needs: a path, the flywheel, the intake, the feed.
     *
     * @param next state to enter
     */
    private void enter(State next) {
        state = next;
        stateTimer.reset();

        switch (next) {
            case SPIN_UP_1:
                shooter.spinUp();
                break;

            case SHOOT_1:
            case SHOOT_2:
            case SHOOT_3:
                shooter.spinUp();
                openFeed();
                // The intake pushes balls up into the feed while shooting.
                intake.intake();
                break;

            case DRIVE_TO_FLOWER_1:
                closeFeed();
                intake.stop();
                shooter.stop();
                follower.follow(toFlower1);
                break;

            case INTAKE_FLOWER_1_FORWARD:
                intake.intake();
                follower.follow(flower1Forward);
                break;

            case INTAKE_FLOWER_1_BACK:
                follower.follow(flower1Back);
                break;

            case DRIVE_TO_SHOOT_POSITION_2:
                intake.stop();
                // Spin up during the drive so the shot is ready on arrival.
                shooter.spinUp();
                follower.follow(flower1ToShootPosition2);
                break;

            case DRIVE_TO_FLOWER_2:
                closeFeed();
                intake.stop();
                shooter.stop();
                follower.follow(toFlower2);
                break;

            case INTAKE_FLOWER_2_FORWARD:
                intake.intake();
                follower.follow(flower2Forward);
                break;

            case INTAKE_FLOWER_2_BACK:
                follower.follow(flower2Back);
                break;

            case RETURN_TO_SHOOT_POSITION_2:
                intake.stop();
                // Spin up during the drive so the shot is ready on arrival.
                shooter.spinUp();
                follower.follow(flower2ToShootPosition2);
                break;

            case DONE:
                closeFeed();
                intake.stop();
                shooter.stop();
                break;

            default:
                break;
        }
    }

    /**
     * @param timeoutSeconds how long to wait before giving up on the path
     * @return {@code true} once the follower reaches the end of the current
     *         path, or the timeout expires
     */
    private boolean pathFinished(double timeoutSeconds) {
        return !follower.isBusy() || stateTimer.seconds() >= timeoutSeconds;
    }

    /**
     * The creep-back is done when the path ends and the intake has had a
     * moment to settle the last ball.
     *
     * @return {@code true} when it is safe to leave the flower
     */
    private boolean creepBackFinished() {
        return pathFinished(Timing.CREEP_TIMEOUT_SECONDS)
                && stateTimer.seconds() >= Timing.INTAKE_SETTLE_SECONDS;
    }

    /**
     * @return {@code true} once the feed has been open long enough for
     *         every ball on board to fire
     */
    private boolean shotFinished() {
        return stateTimer.seconds() >= RobotConstants.Shooter.SHOOT_SECONDS;
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
        telemetry.addData("State", state);
        telemetry.addData("State time (s)", "%.1f", stateTimer.seconds());
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
