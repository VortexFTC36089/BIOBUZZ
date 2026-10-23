package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * Registers Pedro 3 AutoTune procedures.
 *
 * <p>After deploying, connect to the Robot Controller Wi-Fi and open
 * {@code http://192.168.43.1:10158} (Control Hub) or the address shown in
 * the official Pedro tuning docs. Pick a procedure, run it, then paste the
 * generated Java into {@link Constants}.
 *
 * <p>Recommended order: Mecanum directions, then Pinpoint, then Foresight,
 * then Tests. Do not skip localization before Foresight — the algorithm
 * tuner drives the robot using the localizer.
 *
 * @see <a href="https://pedropathing.com/docs/pathing/tuning">Pedro tuning</a>
 */
public class Tuning {

    /**
     * Spins each mecanum motor so you can confirm HardwareMap names and
     * forward/reverse directions.
     *
     * @return the official Mecanum AutoTune procedure
     */
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    /**
     * Measures Pinpoint pod directions and offsets.
     *
     * @return the official Pinpoint AutoTune procedure
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.365852175735113);
        c.yPodOffset.set(3.794639166884535);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    /**
     * Identifies Foresight velocities, braking, and feedback gains.
     *
     * <p>Uses this team's {@link Constants} so the tuner drives the same
     * hardware TeleOp will use.
     *
     * @return the official Foresight AutoTune procedure
     */
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }

    /**
     * Manual drive, pose, hold, line, and curve checks after tuning.
     *
     * @return the official Tests procedure wired to this robot
     */
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig)
        );
    }
}
