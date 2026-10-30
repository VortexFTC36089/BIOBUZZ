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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Sample Autonomous built with Pedro Pathing 3 and Ivy (official command library).
 *
 * <p>Based on the official Pedro Pathing 3 Ivy autonomous example:
 * {@code https://pedropathing.com/docs/pathing/guide/auto-usage}.
 *
 * <p>Uses poses from {@link RobotConstants.PedroPoints}:
 * starts at {@code START}, drives to {@code SCORE}, then drives to {@code PARK}.
 */
@Autonomous(name = "Sample Auto (Ivy)", group = RobotConstants.OpModeGroups.AUTO)
public class SampleAuto extends OpMode {

    private Follower follower;

    // Poses from RobotConstants.PedroPoints
    private final Pose startPose = RobotConstants.PedroPoints.START;
    private final Pose scorePose = RobotConstants.PedroPoints.SCORE;
    private final Pose parkPose = RobotConstants.PedroPoints.PARK;

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

    /**
     * Full autonomous sequence using Ivy's {@code sequential()} group.
     * The second {@code follow()} begins automatically after the first path finishes.
     */
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // TODO(team): add mechanism commands here (e.g., score / intake / servo)
                follow(follower, scoreToPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

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

        Pose pose = follower.pose();
        telemetry.addData("X (in)", pose.x());
        telemetry.addData("Y (in)", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (follower != null) {
            follower.manual(0.0, 0.0, 0.0);
            follower.update();
        }
    }
}
