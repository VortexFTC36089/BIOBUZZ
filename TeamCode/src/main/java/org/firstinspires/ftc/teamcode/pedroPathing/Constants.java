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
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.4384011997012642);
        c.yPodOffset.set(5.275469201756274);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


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
                Controller primaryTranslationalForward = Controller.proportional(0.17213939039122256);
                Controller secondaryTranslationalForward = Controller.proportional(0.06360090510033253);
                Controller primaryTranslationalLateral = Controller.proportional(0.2474019918114243);
                Controller secondaryTranslationalLateral = Controller.proportional(0.091408425271349);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012592468083991752));
                c.brake.set(Controller.proportionalFeedforward(0.01070359787139299));

                c.headingFeedback.set(Controller.proportional(2.827726070099874));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04522120894819609, 0.003986354471637821));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09389388093639425, 0.07088437480518334));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.082695561039119E-4, 9.876324654492466E-4));

                c.maxAchievableForwardVelocity.set(80.09369384506059);
                c.maxAchievableStrafeVelocity.set(67.51221882438458);
                c.naturalForwardDeceleration.set(38.106538766930356);
                c.naturalStrafeDeceleration.set(62.574551827240306);
            }
    );


    /**
     * Builds a follower for this robot using the configs above.
     *
     * @param h FTC hardware map from the running OpMode
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
