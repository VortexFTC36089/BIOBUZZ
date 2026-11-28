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
    private final Pose bOTTOMSHOOT = poseFactory.of(58, 14, 90);
    private final Pose iNTAKEGARDEN = poseFactory.of(13, 10, 178.3634);
    private final Pose iNTAKEGARDENControl1 = poseFactory.of(48, 9, 0);
    private final Pose tOPSHOOTStart = poseFactory.of(13, 10, 180);
    private final Pose tOPSHOOT = poseFactory.of(64, 130, 270);
    private final Pose tOPSHOOTControl1 = poseFactory.of(19, 78, 0);
    private final Pose tOPSHOOTControl2 = poseFactory.of(38, 103, 0);
    private final Pose tOPFLOWERStart = poseFactory.of(64, 130, 270);
    private final Pose tOPFLOWER = poseFactory.of(54, 120, 90);
    private final Pose tOPFLOWERControl1 = poseFactory.of(54, 94, 0);
    private final Pose fLOWERINTAKE = poseFactory.of(54, 125, 90);
    private final Pose tOPSHOOT_2Start = poseFactory.of(54, 125, 90);
    private final Pose tOPSHOOT_2 = poseFactory.of(64, 130, 270);
    private final Pose tOPSHOOT_2Control1 = poseFactory.of(53, 97, 0);
    private final Pose pARKStart = poseFactory.of(64, 130, 270);
    private final Pose pARK = poseFactory.of(17, 93, 270);
    private final Pose pARKControl1 = poseFactory.of(61, 121, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, bOTTOMSHOOT()),
                follow(follower, iNTAKEGARDEN()),
                follow(follower, tOPSHOOT()),
                follow(follower, tOPFLOWER()),
                follow(follower, fLOWERINTAKE()),
                follow(follower, tOPSHOOT_2()),
                follow(follower, pARK())
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

    public Path bOTTOMSHOOT() {
        return Paths.line(start, bOTTOMSHOOT).constant(bOTTOMSHOOT);
    }

    public Path iNTAKEGARDEN() {
        return Paths.curve(bOTTOMSHOOT, iNTAKEGARDENControl1, iNTAKEGARDEN).tangent();
    }

    public Path tOPSHOOT() {
        return Paths.curve(tOPSHOOTStart, tOPSHOOTControl1, tOPSHOOTControl2, tOPSHOOT).linear(tOPSHOOTStart, tOPSHOOT);
    }

    public Path tOPFLOWER() {
        return Paths.curve(tOPFLOWERStart, tOPFLOWERControl1, tOPFLOWER).linear(tOPFLOWERStart, tOPFLOWER);
    }

    public Path fLOWERINTAKE() {
        return Paths.line(tOPFLOWER, fLOWERINTAKE).constant(fLOWERINTAKE);
    }

    public Path tOPSHOOT_2() {
        return Paths.curve(tOPSHOOT_2Start, tOPSHOOT_2Control1, tOPSHOOT_2).linear(tOPSHOOT_2Start, tOPSHOOT_2);
    }

    public Path pARK() {
        return Paths.curve(pARKStart, pARKControl1, pARK).linear(pARKStart, pARK);
    }
}
