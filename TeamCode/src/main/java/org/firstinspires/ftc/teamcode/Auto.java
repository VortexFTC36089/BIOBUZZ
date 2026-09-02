package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private Follower follower;

    Constants constants = new Constants();
    private PathChain action0, action1, action2, action3;

    public void buildPaths() {
        action0 = follower.pathBuilder()
                .addPath(new BezierLine(constants.startPose, constants.pose1))
                .setLinearHeadingInterpolation(constants.startPose.getHeading(), constants.pose1.getHeading())
                .build();

        action1 = follower.pathBuilder()
                .addPath(new BezierLine(constants.pose1, constants.pose2))
                .setLinearHeadingInterpolation(constants.pose1.getHeading(), constants.pose2.getHeading())
                .build();

        action2 = follower.pathBuilder()
                .addPath(new BezierLine(constants.pose2, constants.pose3))
                .setLinearHeadingInterpolation(constants.pose2.getHeading(), constants.pose3.getHeading())
                .build();

        action3 = follower.pathBuilder()
                .addPath(new BezierLine(constants.pose3, constants.pose4))
                .setLinearHeadingInterpolation(constants.pose3.getHeading(), constants.pose4.getHeading())
                .build();
    }

    public Command autoRoutine() {
        return sequential (
                follow(follower, action0, true),
                follow(follower, action1, true),
                follow(follower, action2),
                follow(follower, action3)
        );

        // Add additional commands here
    }

    @Override
    public void runOpMode() {
        //These will run when the OpMode is initiated
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(constants.startPose);
        waitForStart();
        //We schedule all our commands when we start the OpMode
        schedule(autoRoutine());
        while (opModeIsActive()) {
            //Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();
            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
