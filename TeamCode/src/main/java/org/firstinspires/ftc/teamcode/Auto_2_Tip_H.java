package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
@Autonomous(name = "Auto_2_Tip_H", group = "Autonomous")
public class Auto_2_Tip_H extends LinearOpMode {

    // Path points, editable in Panels (Auto_2_Tip_H) before Play. X and Y are Pedro field
    // inches, headings are degrees. A CONTROL point only shapes a curve. Each path starts
    // where the one before it ends. Panels edits are NOT saved to this file.

    public static double START_X = 56;
    public static double START_Y = 8;
    public static double START_HEADING_DEG = 90;

    public static double BOTTOM_SHOOT_X = 58;
    public static double BOTTOM_SHOOT_Y = 14;
    public static double BOTTOM_SHOOT_HEADING_DEG = 90;

    /** The robot faces along the curve on the way here, so there is no heading to set. */
    public static double INTAKE_GARDEN_X = 13;
    public static double INTAKE_GARDEN_Y = 10;
    public static double INTAKE_GARDEN_CONTROL1_X = 48;
    public static double INTAKE_GARDEN_CONTROL1_Y = 9;

    /** Heading the drive from the intake garden to the top shot starts from. */
    public static double TOP_SHOOT_START_HEADING_DEG = 180;
    public static double TOP_SHOOT_X = 64;
    public static double TOP_SHOOT_Y = 130;
    public static double TOP_SHOOT_HEADING_DEG = 270;
    public static double TOP_SHOOT_CONTROL1_X = 19;
    public static double TOP_SHOOT_CONTROL1_Y = 78;
    public static double TOP_SHOOT_CONTROL2_X = 38;
    public static double TOP_SHOOT_CONTROL2_Y = 103;

    public static double TOP_FLOWER_X = 54;
    public static double TOP_FLOWER_Y = 120;
    public static double TOP_FLOWER_HEADING_DEG = 90;
    public static double TOP_FLOWER_CONTROL1_X = 54;
    public static double TOP_FLOWER_CONTROL1_Y = 94;

    public static double FLOWER_INTAKE_X = 54;
    public static double FLOWER_INTAKE_Y = 125;
    public static double FLOWER_INTAKE_HEADING_DEG = 90;

    public static double TOP_SHOOT_2_X = 64;
    public static double TOP_SHOOT_2_Y = 130;
    public static double TOP_SHOOT_2_HEADING_DEG = 270;
    public static double TOP_SHOOT_2_CONTROL1_X = 53;
    public static double TOP_SHOOT_2_CONTROL1_Y = 97;

    public static double PARK_X = 17;
    public static double PARK_Y = 93;
    public static double PARK_HEADING_DEG = 270;
    public static double PARK_CONTROL1_X = 61;
    public static double PARK_CONTROL1_Y = 121;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose start;
    private Pose bOTTOMSHOOT;
    private Pose iNTAKEGARDEN;
    private Pose iNTAKEGARDENControl1;
    private Pose tOPSHOOTStart;
    private Pose tOPSHOOT;
    private Pose tOPSHOOTControl1;
    private Pose tOPSHOOTControl2;
    private Pose tOPFLOWERStart;
    private Pose tOPFLOWER;
    private Pose tOPFLOWERControl1;
    private Pose fLOWERINTAKE;
    private Pose tOPSHOOT_2Start;
    private Pose tOPSHOOT_2;
    private Pose tOPSHOOT_2Control1;
    private Pose pARKStart;
    private Pose pARK;
    private Pose pARKControl1;

    /** Start values the follower was last given; NaN until the first time. */
    private double appliedStartX = Double.NaN;
    private double appliedStartY = Double.NaN;
    private double appliedStartHeadingDeg = Double.NaN;

    /** Builds every pose from the values as they are in Panels right now. */
    private void buildPoses() {
        start = poseFactory.of(START_X, START_Y, START_HEADING_DEG);
        bOTTOMSHOOT = poseFactory.of(BOTTOM_SHOOT_X, BOTTOM_SHOOT_Y, BOTTOM_SHOOT_HEADING_DEG);
        iNTAKEGARDEN = poseFactory.of(INTAKE_GARDEN_X, INTAKE_GARDEN_Y, 178.3634);
        iNTAKEGARDENControl1 =
                poseFactory.of(INTAKE_GARDEN_CONTROL1_X, INTAKE_GARDEN_CONTROL1_Y, 0);
        tOPSHOOTStart =
                poseFactory.of(INTAKE_GARDEN_X, INTAKE_GARDEN_Y, TOP_SHOOT_START_HEADING_DEG);
        tOPSHOOT = poseFactory.of(TOP_SHOOT_X, TOP_SHOOT_Y, TOP_SHOOT_HEADING_DEG);
        tOPSHOOTControl1 = poseFactory.of(TOP_SHOOT_CONTROL1_X, TOP_SHOOT_CONTROL1_Y, 0);
        tOPSHOOTControl2 = poseFactory.of(TOP_SHOOT_CONTROL2_X, TOP_SHOOT_CONTROL2_Y, 0);
        tOPFLOWERStart = tOPSHOOT;
        tOPFLOWER = poseFactory.of(TOP_FLOWER_X, TOP_FLOWER_Y, TOP_FLOWER_HEADING_DEG);
        tOPFLOWERControl1 = poseFactory.of(TOP_FLOWER_CONTROL1_X, TOP_FLOWER_CONTROL1_Y, 0);
        fLOWERINTAKE =
                poseFactory.of(FLOWER_INTAKE_X, FLOWER_INTAKE_Y, FLOWER_INTAKE_HEADING_DEG);
        tOPSHOOT_2Start = fLOWERINTAKE;
        tOPSHOOT_2 = poseFactory.of(TOP_SHOOT_2_X, TOP_SHOOT_2_Y, TOP_SHOOT_2_HEADING_DEG);
        tOPSHOOT_2Control1 =
                poseFactory.of(TOP_SHOOT_2_CONTROL1_X, TOP_SHOOT_2_CONTROL1_Y, 0);
        pARKStart = tOPSHOOT_2;
        pARK = poseFactory.of(PARK_X, PARK_Y, PARK_HEADING_DEG);
        pARKControl1 = poseFactory.of(PARK_CONTROL1_X, PARK_CONTROL1_Y, 0);
    }

    /**
     * Re-reads the points from Panels and, the first time and whenever the START values
     * were edited, gives the follower the new start pose.
     */
    private void syncStartPose() {
        buildPoses();
        if (START_X != appliedStartX
                || START_Y != appliedStartY
                || START_HEADING_DEG != appliedStartHeadingDeg) {
            follower.setPose(start);
            follower.update();
            appliedStartX = START_X;
            appliedStartY = START_Y;
            appliedStartHeadingDeg = START_HEADING_DEG;
        }
    }

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
        syncStartPose();

        // While waiting for Play, pick up points edited in Panels.
        while (opModeInInit()) {
            syncStartPose();
            telemetry.addLine("Path points: edit them in Panels (Auto_2_Tip_H) before Play.");
            telemetry.addData("Start X / Y / Heading", "%.1f in / %.1f in / %.1f deg",
                    START_X, START_Y, START_HEADING_DEG);
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            return;
        }
        // Final read of the Panels values: every path is built from them right now.
        syncStartPose();
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
