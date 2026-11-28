package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto_2_Tip_H", group = "Autonomous")
public class Auto_2_Tip_H extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(58, 14, 90);
    private final Pose point2 = poseFactory.of(72.6586, 17.617, 180);
    private final Pose point3 = poseFactory.of(12.9549, 9.4983, 179.5693);
    private final Pose point3Control1 = poseFactory.of(47.9174, 9.2355, 0);
    private final Pose point4Start = poseFactory.of(12.9549, 9.4983, 180);
    private final Pose point4 = poseFactory.of(58, 130, 270);
    private final Pose point4Control1 = poseFactory.of(18.5768, 78.4038, 0);
    private final Pose point4Control2 = poseFactory.of(37.6936, 102.8587, 0);
    private final Pose point5Start = poseFactory.of(58, 130, 270);
    private final Pose point5 = poseFactory.of(50, 120, 90);
    private final Pose point5Control1 = poseFactory.of(54.377, 94.3724, 0);
    private final Pose point6 = poseFactory.of(49.787, 125, 90);
    private final Pose point7Start = poseFactory.of(49.787, 125, 90);
    private final Pose point7 = poseFactory.of(58, 130, 270);
    private final Pose point7Control1 = poseFactory.of(53.1328, 96.9509, 0);
    private final Pose point8Start = poseFactory.of(58, 130, 270);
    private final Pose point8 = poseFactory.of(11.7423, 121.3687, 270);
    private final Pose point8Control1 = poseFactory.of(61.1438, 121.0953, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).tangent();
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.curve(point8Start, point8Control1, point8).linear(point8Start, point8);
    }
}
