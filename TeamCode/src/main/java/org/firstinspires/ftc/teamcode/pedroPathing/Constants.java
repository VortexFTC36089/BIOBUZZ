package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
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
     * Prevents anyone from constructing this holder class.
     */
    private Constants() {
        // Constants + factory only.
    }

    /**
     * Mecanum motor names and directions.
     *
     * <p>Left-side REVERSE / right-side FORWARD is the usual mecanum starting
     * point so "forward power" actually drives toward the intake/front.
     * Confirm with the Mecanum AutoTuner before trusting it.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set(RobotConstants.Drive.FRONT_LEFT);
                c.frontRightName.set(RobotConstants.Drive.FRONT_RIGHT);
                c.backLeftName.set(RobotConstants.Drive.BACK_LEFT);
                c.backRightName.set(RobotConstants.Drive.BACK_RIGHT);

                // TODO(team): verify direction with Mecanum AutoTuner
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                // Brake mode holds the robot when TeleOp sticks return to zero.
                c.manualBrakeMode.set(true);
            }
    );

    /**
     * goBILDA Pinpoint localizer.
     *
     * <p>Offsets are inches from the robot center of rotation. Zeros compile
     * but will report the wrong pose until AutoTune (or a tape measure)
     * fills them in. Do not copy another team's offsets.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set(RobotConstants.Localizer.PINPOINT);

                // TODO(team): confirm 4-bar vs swingarm vs custom pods
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

                // TODO(team): tune — run Pinpoint AutoTuner or measure from robot center
                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);

                // TODO(team): verify direction with Pinpoint AutoTuner
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
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
                // Official example starting gains from pedropathing.com Constants.
                // TODO(team): replace this entire block with Foresight AutoTune output
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(
                        Controller.piecewise(secondaryTranslationalForward)
                                .put(2.5, primaryTranslationalForward)
                );
                c.strafeTranslational.set(
                        Controller.piecewise(secondaryTranslationalLateral)
                                .put(2.5, primaryTranslationalLateral)
                );
            }
    );

    /**
     * Builds a follower for this robot using the configs above.
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     * @return a follower that owns drive motors, Pinpoint, and Foresight
     */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    /**
     * Alias for {@link #create(HardwareMap)} so older Pedro 2.x examples
     * that call {@code createFollower} still compile against this file.
     *
     * @param hardwareMap FTC hardware map from the running OpMode
     * @return the same follower {@link #create(HardwareMap)} would return
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return create(hardwareMap);
    }
}
