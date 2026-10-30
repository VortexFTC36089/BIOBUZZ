package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Sample Autonomous built with Pedro Pathing 3 and Ivy (official command library).
 *
 * <p>Based on the official Pedro Pathing 3 Ivy autonomous example:
 * {@code https://pedropathing.com/docs/pathing/guide/auto-usage}.
 *
 * <p>Uses poses from {@link RobotConstants.PedroPoints}:
 * starts at {@code START}, drives to {@code SCORE}, shoots, then drives to {@code PARK}.
 */
@Autonomous(name = "Sample Auto (Ivy)", group = RobotConstants.OpModeGroups.AUTO)
public class SampleAuto extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    // Poses from RobotConstants.PedroPoints
    private final Pose startPose = RobotConstants.PedroPoints.START;
    private final Pose scorePose = RobotConstants.PedroPoints.SCORE;
    private final Pose parkPose = RobotConstants.PedroPoints.PARK;

    // ---- Path methods ----

    /**
     * Straight line from START to SCORE with linear heading interpolation.
     */
    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }

    /**
     * Straight line from SCORE to PARK with linear heading interpolation.
     */
    private Path scoreToPark() {
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }

    // ---- Mechanism Ivy commands ----

    /**
     * Starts the intake motor.
     */
    private Command runIntake() {
        return instant(() -> intake.intake());
    }

    /**
     * Stops the intake motor.
     */
    private Command stopIntake() {
        return instant(() -> intake.stop());
    }

    /**
     * Spins the shooter flywheel up to the target RPM for the current distance
     * to the hive, waits until the flywheel reaches speed, feeds the game piece
     * with the intake for 500 ms, then stops both the intake and shooter.
     */
    private Command shoot() {
        return sequential(
                instant(() -> {
                    Pose robot = follower.pose();
                    boolean leftHalf = robot.y() >= RobotConstants.Hive.FIELD_MIDLINE_Y_INCHES;
                    Pose hive = leftHalf
                            ? new Pose(RobotConstants.Hive.LEFT_HIVE_X_INCHES, RobotConstants.Hive.LEFT_HIVE_Y_INCHES)
                            : new Pose(RobotConstants.Hive.RIGHT_HIVE_X_INCHES, RobotConstants.Hive.RIGHT_HIVE_Y_INCHES);
                    shooter.setTargetFromDistance(robot.distance(hive));
                }),
                waitUntil(shooter::atSpeed),
                runIntake(),
                waitMs(500),
                stopIntake(),
                instant(() -> shooter.stop())
        );
    }

    // ---- Autonomous routine ----

    /**
     * Full autonomous sequence using Ivy's {@code sequential()} and {@code parallel()} groups.
     */
    private Command autoRoutine() {
        return sequential(
                parallel(
                        follow(follower, startToScore()),
                        runIntake()
                ),
                stopIntake(),
                shoot(),
                follow(follower, scoreToPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        telemetry.addLine("Sample Auto (Ivy) initialized");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        intake.update();
        shooter.update();

        Pose pose = follower.pose();
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRpm(), shooter.getTargetRpm());
        telemetry.update();
    }

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
            follower.manual(0.0, 0.0, 0.0);
            follower.update();
        }
    }
}
