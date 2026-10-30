package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's last known pose from autonomous to TeleOp.
 *
 * <p>Static fields survive between OpModes while the app stays running, so
 * {@link MainAutonomous} writes here as it ends and a TeleOp can read it to
 * start field-centric driving with the correct heading instead of 0.
 */
public final class PoseStorage {

    private PoseStorage() {
        // Holder class only.
    }

    /**
     * Pose reported by the follower when the last autonomous stopped, or
     * {@code null} if no autonomous has run since the app started.
     */
    public static Pose lastAutoPose = null;
}
