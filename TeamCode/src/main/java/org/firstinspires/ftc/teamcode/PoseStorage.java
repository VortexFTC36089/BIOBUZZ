package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's last known pose from one OpMode to the next.
 *
 * <p>Static fields survive between OpModes while the Robot Controller app
 * stays running. {@link MainAuto} and {@link SecondAuto} write
 * {@link #lastAutoPose} and {@link #lastAutoTurretZeroTicks} as they run, and
 * {@link MainTeleOp} reads them to start the follower at the exact pose where
 * autonomous finished and to keep the turret's zero.
 *
 * <p>The turret aims from this pose, so a wrong pose here means a wrong aim
 * for the whole TeleOp. Main TeleOp shows the pose it is about to use on the
 * init screen and lets the driver pick a different one before Play.
 */
public final class PoseStorage {

    private PoseStorage() {
        // Holder class only.
    }

    /**
     * Pose reported by the follower when the last autonomous ran/stopped, or
     * {@code null} if no autonomous has run since the app started or since a
     * TeleOp last used it. Main TeleOp clears it on Play, so an old autonomous
     * pose is never reused by a later practice run.
     */
    public static Pose lastAutoPose = null;

    /**
     * Raw turret-encoder count that meant "turret straight forward" on the last
     * autonomous: the last sample taken on its init screen, which is the moment
     * Play was pressed if it was pressed (the turret has been lined up by hand by
     * then), or {@code null} under the same conditions as {@link #lastAutoPose}.
     *
     * <p>The hub keeps counting the encoder between OpModes, so TeleOp restores this
     * zero instead of taking a new one. That way TeleOp knows where the turret
     * really points even if it was turned or knocked during autonomous. Only
     * valid while the hub has stayed powered.
     */
    public static Integer lastAutoTurretZeroTicks = null;

    /**
     * Pose reported by the follower on the most recent Main TeleOp loop, or
     * {@code null} if TeleOp has not run since the app started. If TeleOp is
     * stopped and re-initialized in the middle of a match, the driver can
     * resume from here on the init screen instead of starting from the default
     * start pose.
     */
    public static Pose lastTeleOpPose = null;

    /**
     * Raw turret-encoder count that the most recent Main TeleOp treated as
     * "turret straight forward", or {@code null} if TeleOp has not been played
     * since the app started. The hub keeps counting between OpModes, so after a
     * TeleOp restart in the middle of a match (when the turret is not pointing
     * forward and nobody may touch it) this zero is still right. Only valid
     * while the hub has stayed powered.
     */
    public static Integer lastTeleOpTurretZeroTicks = null;
}
