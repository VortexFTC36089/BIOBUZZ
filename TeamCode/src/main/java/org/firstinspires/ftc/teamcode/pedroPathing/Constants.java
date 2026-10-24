package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Pedro Pathing 3 robot configuration.
 *
 * <p>This is the only file the follower reads for drivetrain names, Pinpoint
 * offsets, and Foresight (path-following) gains. OpModes should call
 * {@link #create(HardwareMap)} and not construct a {@link Follower} themselves.
 *
 * <p>Hardware this file assumes:
 * <ul>
 *   <li>Four mecanum motors named in {@link RobotConstants.Drive}</li>
 *   <li>A goBILDA Pinpoint named in {@link RobotConstants.Localizer}</li>
 * </ul>
 *
 * <p>Pedro 3 no longer uses {@code FollowerBuilder} / {@code FollowerConstants}
 * from 2.x. The current constructor is
 * {@code new Follower(localizer, drivetrain, foresight)}.
 *
 * <p>Coordinate system (from official Pedro docs):
 * <ul>
 *   <li>+X is forward, +Y is left, heading is radians, counterclockwise positive</li>
 *   <li>{@code follower.pose()} x/y are inches</li>
 * </ul>
 */
public final class Constants {


    /**
     * Mecanum motor names and directions.
     *
     * <p>Left-side REVERSE / right-side FORWARD is the usual mecanum starting
     * point so "forward power" actually drives toward the intake/front.
     * Confirm with the Mecanum AutoTuner before trusting it.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front");
        c.frontRightName.set("right_front");
        c.backLeftName.set("left_back");
        c.backRightName.set("right_back");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    /**
     * goBILDA Pinpoint localizer.
     *
     * <p>Offsets are inches from the robot center of rotation. Zeros compile
     * but will report the wrong pose until AutoTune (or a tape measure)
     * fills them in. Do not copy another team's offsets.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(2.187); // TODO(team): tune
                c.yPodOffset.set(-4.572); // TODO(team): tune
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
    );


    /**
     * Foresight path-following algorithm.
     *
     * <p>Only the official starting proportional gains are set here. Brake
     * coefficients and max velocities are robot-specific; inventing them
     * makes autonomous worse, not better. Run Foresight AutoTune and paste
     * the generated block over this config.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
                c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
                c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
                c.headingFeedback.set(Controller.proportional(5.258721785960744));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
                c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
                c.maxAchievableForwardVelocity.set(72.72923108818539);
                c.maxAchievableStrafeVelocity.set(52.34323936525474);
                c.naturalForwardDeceleration.set(85.01144677379789);
                c.naturalStrafeDeceleration.set(104.49787535782846);
            }
    );


    /**
     * Builds a follower for this robot using the configs above.
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     * @return a follower that owns drive motors, Pinpoint, and Foresight
     */
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
