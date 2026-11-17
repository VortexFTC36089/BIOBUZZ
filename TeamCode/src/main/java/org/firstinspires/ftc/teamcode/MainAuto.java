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

@Autonomous(name = RobotConstants.OpModeNames.AUTO_BOTTOM, group = "Autonomous",
        preselectTeleOp = RobotConstants.OpModeNames.TELEOP_AFTER_AUTO)
public class MainAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose  path1 = poseFactory.of(56, 28.059, 180);

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
        // Hand-off to TeleOp starts at the auto start pose, in case auto is
        // stopped before it ever moves.
        blackboard.put(RobotBlackboard.LAST_AUTO_POSE, start);

        // Turret hand-off to TeleOp. While waiting for Play, keep recording the
        // turret encoder count as "turret straight forward", so the zero is taken
        // at the moment Play is pressed. The turret is not driven in autonomous,
        // but the hub keeps counting, so TeleOp restores this zero and knows where
        // the turret really points even if it moved during autonomous.
        while (opModeInInit()) {
            int turretTicks = TurretSubsystem.readEncoderTicks(hardwareMap);
            if (isStopRequested()) {
                // Stop interrupts the hub read and the SDK then returns 0 instead of
                // the real count. Keep the last good count.
                break;
            }
            blackboard.put(RobotBlackboard.LAST_AUTO_TURRET_ZERO_TICKS, turretTicks);

            telemetry.addLine("AutoPath Bottom ready");
            telemetry.addLine("Point the turret STRAIGHT FORWARD now. Its zero is taken when you press Play.");
            telemetry.addData("Turret encoderTicks (live)", turretTicks);
            telemetry.addData("TeleOp after this auto", RobotConstants.OpModeNames.TELEOP_AFTER_AUTO);
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            return;
        }
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            // Save the latest pose every loop so TeleOp (and its turret aiming)
            // starts from wherever autonomous ended, even if it is stopped
            // early or times out.
            // A Stop (button or the 30 second timer) interrupts this thread, and the
            // Pinpoint read can then come back as all zeros. Do not let that
            // overwrite the last good pose.
            if (!isStopRequested()) {
                blackboard.put(RobotBlackboard.LAST_AUTO_POSE, follower.pose());
            }
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
        return Paths.line(start, path1).linear(start, path1);
    }
}
