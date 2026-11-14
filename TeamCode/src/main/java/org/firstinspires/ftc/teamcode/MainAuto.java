package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous path OpMode built with Pedro Pathing 3 and Ivy.
 *
 * <p>Starts at {@link RobotConstants.Field#START_POSE}, follows {@link #path1()},
 * and stores the live pose in {@link PoseStorage#lastAutoPose} so
 * {@link MainTeleOp} knows where the robot ended.
 */
@Autonomous(name = "AutoPath", group = RobotConstants.OpModeGroups.AUTO, preselectTeleOp = "Main TeleOp")
public class MainAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = RobotConstants.Field.START_POSE;
    private final Pose path1 = poseFactory.of(56, 28.059, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        schedule(autoRoutine());

        try {
            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                // Hand the live pose to TeleOp so the turret aims from where auto really ended.
                PoseStorage.lastAutoPose = follower.pose();

                telemetry.addData("X (in)", "%.1f", follower.pose().x());
                telemetry.addData("Y (in)", "%.1f", follower.pose().y());
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", "%.1f", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
            }
        } finally {
            Scheduler.reset();
            if (follower != null) {
                PoseStorage.lastAutoPose = follower.pose();
                follower.manual(0.0, 0.0, 0.0);
                follower.update();
            }
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }
}
