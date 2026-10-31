package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's last known pose from autonomous to TeleOp.
 *
 * <p>Static fields survive between OpModes while the Robot Controller app
 * stays running, so {@link MainAuto} writes here as it runs/ends and
 * {@link MainTeleOp} reads it to initialize the follower at the exact pose
 * where autonomous finished.
 */
public final class PoseStorage {

    private PoseStorage() {
        // Holder class only.
    }

    /**
     * Pose reported by the follower when the last autonomous ran/stopped, or
     * {@code null} if no autonomous has run since the app started.
     */
    public static Pose lastAutoPose = null;
}
