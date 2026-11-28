package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Works out the turn power that points the whole robot at the hive. It only does math: the
 * OpMode sends the returned power to the follower as its manual turn command.
 *
 * <p>It aims like the turret does: at the top hive side in the top half of the field and at
 * the bottom side in the bottom half, using the same hive coordinates, switch line and
 * hysteresis from {@link RobotConstants.Turret}. The aim is worked out from the Pinpoint
 * position with the chassis's own offsets, not the turret's pivot offsets. The heading it
 * asks for puts the hive straight ahead of the offset aim point. The offsets, trim, gains
 * and tolerances are in {@link RobotConstants.ChassisAlign}.
 */
public class ChassisAligner {

    private boolean topSide;
    private boolean onTarget;
    private boolean firstUpdate = true;
    private double errorRad;
    private double previousErrorRad;
    private long previousNanos;

    /**
     * Call when alignment is switched on.
     *
     * @param startOnTopSide true to start on the top hive side, false for the bottom side
     */
    public void start(boolean startOnTopSide) {
        topSide = startOnTopSide;
        onTarget = false;
        firstUpdate = true;
        errorRad = 0.0;
    }

    /**
     * Call once per loop while aligning.
     *
     * @param pose current robot pose from the follower
     * @return turn power, positive = counterclockwise; 0 while aligned
     */
    public double turnPower(Pose pose) {
        long now = System.nanoTime();
        double dtSeconds = (now - previousNanos) / 1e9;
        previousNanos = now;
        return turnPower(pose, dtSeconds);
    }

    /** {@link #turnPower(Pose)} with the loop time passed in. */
    double turnPower(Pose pose, double dtSeconds) {
        selectSide(pose.y());

        double error = wrap(headingToHive(pose.x(), pose.y(), pose.heading(), topSide)
                - pose.heading());
        if (Double.isNaN(error)) {
            // A bad pose must not turn the robot.
            onTarget = false;
            firstUpdate = true;
            return 0.0;
        }

        // How fast the error is changing, for damping. Skipped on the first loop, after a
        // side switch and after a stalled loop, where it would be meaningless.
        double errorRate = (!firstUpdate && dtSeconds > 1e-4 && dtSeconds <= 0.5)
                ? wrap(error - previousErrorRad) / dtSeconds : 0.0;
        firstUpdate = false;
        previousErrorRad = error;
        errorRad = error;

        // Aligned, with hysteresis so the robot does not twitch around the target.
        double enter = Math.toRadians(RobotConstants.ChassisAlign.TOLERANCE_DEGREES);
        double exit = Math.max(
                Math.toRadians(RobotConstants.ChassisAlign.EXIT_TOLERANCE_DEGREES), enter);
        double absError = Math.abs(error);
        onTarget = absError <= enter || (onTarget && absError <= exit);
        if (onTarget) {
            return 0.0;
        }

        double power = RobotConstants.ChassisAlign.KP * error
                + RobotConstants.ChassisAlign.KD * errorRate
                + RobotConstants.ChassisAlign.KS * Math.signum(error);
        double max = clip(RobotConstants.ChassisAlign.MAX_TURN_POWER, 0.0, 1.0);
        return clip(power, -max, max);
    }

    /**
     * Aims from a point on the robot placed by
     * {@link RobotConstants.ChassisAlign#OFFSET_X_INCHES} and
     * {@link RobotConstants.ChassisAlign#OFFSET_Y_INCHES}. That point moves as the robot
     * turns, so call this again with each new pose.
     *
     * @param x       Pinpoint X, inches
     * @param y       Pinpoint Y, inches
     * @param heading Pinpoint heading, radians
     * @param top     true for the top hive side, false for the bottom side
     * @return the field heading (radians) toward the hive side from that point, including
     *         {@link RobotConstants.ChassisAlign#TRIM_DEGREES}
     */
    public static double headingToHive(double x, double y, double heading, boolean top) {
        double hiveX = top
                ? RobotConstants.Turret.TOP_TARGET_X_INCHES
                : RobotConstants.Turret.BOTTOM_TARGET_X_INCHES;
        double hiveY = top
                ? RobotConstants.Turret.TOP_TARGET_Y_INCHES
                : RobotConstants.Turret.BOTTOM_TARGET_Y_INCHES;
        // Robot frame -> field frame, the same way TurretSubsystem places its pivot.
        double offsetX = RobotConstants.ChassisAlign.OFFSET_X_INCHES;
        double offsetY = RobotConstants.ChassisAlign.OFFSET_Y_INCHES;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double aimX = x + offsetX * cos - offsetY * sin;
        double aimY = y + offsetX * sin + offsetY * cos;
        return wrap(Math.atan2(hiveY - aimY, hiveX - aimX)
                + Math.toRadians(RobotConstants.ChassisAlign.TRIM_DEGREES));
    }

    private void selectSide(double y) {
        double mid = RobotConstants.Turret.SIDE_SWITCH_Y_INCHES;
        double band = RobotConstants.Turret.SIDE_HYSTERESIS_INCHES;
        boolean top = topSide;
        if (y > mid + band) {
            top = true;
        } else if (y < mid - band) {
            top = false;
        }
        if (top != topSide) {
            // The target jumps to the other side.
            topSide = top;
            onTarget = false;
            firstUpdate = true;
        }
    }

    /** @return true while aiming at the top hive side */
    public boolean isAimingAtTopSide() {
        return topSide;
    }

    /** @return true while the robot is within tolerance of the aim heading */
    public boolean isAligned() {
        return onTarget;
    }

    /** @return heading still to turn, degrees, positive = counterclockwise */
    public double getErrorDegrees() {
        return Math.toDegrees(errorRad);
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
