package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public final Pose startPose = new Pose(70, 25, Math.toRadians(90));
    public final Pose pose1 = new Pose(70, 40, Math.toRadians(90));
    public final Pose pose2 = new Pose(70, 60, Math.toRadians(90));
    public final Pose pose3 = new Pose(70, 80, Math.toRadians(90));
    public final Pose pose4 = new Pose(70, 100, Math.toRadians(90));
    public final Pose pose5 = new Pose (70, 120, Math.toRadians(90));

    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}