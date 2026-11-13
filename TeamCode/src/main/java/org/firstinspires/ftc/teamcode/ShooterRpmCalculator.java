package org.firstinspires.ftc.teamcode;

/**
 * Calculates the flywheel RPM for a shot from the distance to the target, so no
 * table of measured points is needed.
 *
 * <h3>How it works</h3>
 * <ol>
 *   <li>Projectile motion gives the exit speed a piece needs to leave the shooter at
 *       {@link RobotConstants.Outtake#LAUNCH_ANGLE_DEGREES} and pass through the goal
 *       height at the given distance.</li>
 *   <li>Flywheel RPM is assumed proportional to exit speed. The ratio comes from ONE
 *       test shot: {@link RobotConstants.Outtake#TARGET_RPM} scores from
 *       {@link #CALIBRATION_DISTANCE_INCHES} (the start pose). Every other distance is
 *       scaled from that shot.</li>
 * </ol>
 *
 * <h3>What the team must provide</h3>
 * The launch angle, the launch height, and the goal height in
 * {@link RobotConstants.Outtake}, plus the one calibration shot. If shots land long or
 * short everywhere, retune {@code TARGET_RPM}. If they are right at the start pose but
 * drift as the distance changes, correct the launch angle or the heights.
 */
public final class ShooterRpmCalculator {

    private ShooterRpmCalculator() {
        // Utility class
    }

    /** Gravity in inches per second squared (Pedro distances are in inches). */
    private static final double GRAVITY_INCHES_PER_SEC2 = 386.09;

    private static final boolean START_IN_TOP_HALF =
            RobotConstants.Field.START_POSE.y() > RobotConstants.Turret.SIDE_SWITCH_Y_INCHES;

    /**
     * Distance from the start pose to the hive side it aims at, in inches. This is the
     * distance {@link RobotConstants.Outtake#TARGET_RPM} is tuned for, and it updates
     * automatically when START_POSE or the hive coordinates change.
     */
    public static final double CALIBRATION_DISTANCE_INCHES = Math.hypot(
            (START_IN_TOP_HALF ? RobotConstants.Turret.TOP_TARGET_X_INCHES
                    : RobotConstants.Turret.BOTTOM_TARGET_X_INCHES) - RobotConstants.Field.START_POSE.x(),
            (START_IN_TOP_HALF ? RobotConstants.Turret.TOP_TARGET_Y_INCHES
                    : RobotConstants.Turret.BOTTOM_TARGET_Y_INCHES) - RobotConstants.Field.START_POSE.y());

    /**
     * @param distanceInches horizontal distance from the turret to the target in inches
     * @return flywheel RPM for that distance, limited to
     *         {@link RobotConstants.Outtake#MIN_RPM}..{@link RobotConstants.Outtake#MAX_RPM}
     */
    public static double getRpm(double distanceInches) {
        double calibrationSpeed = requiredExitSpeed(CALIBRATION_DISTANCE_INCHES);
        if (Double.isNaN(calibrationSpeed)) {
            // The geometry constants say the calibration shot itself is impossible, so
            // they are wrong. Use the tuned RPM unchanged rather than scale from nonsense.
            return RobotConstants.Outtake.TARGET_RPM;
        }

        double speed = requiredExitSpeed(distanceInches);
        if (Double.isNaN(speed)) {
            // Too close: at this launch angle the piece cannot climb to the goal height
            // before it gets there. Full speed is the nearest thing to a shot.
            return RobotConstants.Outtake.MAX_RPM;
        }

        double rpm = RobotConstants.Outtake.TARGET_RPM * speed / calibrationSpeed;
        return Math.max(RobotConstants.Outtake.MIN_RPM, Math.min(RobotConstants.Outtake.MAX_RPM, rpm));
    }

    /**
     * Exit speed (inches per second) a piece needs to pass through the goal height at
     * the given distance when launched at the fixed launch angle, ignoring air drag.
     *
     * @return the speed, or {@code NaN} if no speed reaches the goal from that distance
     */
    private static double requiredExitSpeed(double distanceInches) {
        double angle = Math.toRadians(RobotConstants.Outtake.LAUNCH_ANGLE_DEGREES);
        double rise = RobotConstants.Outtake.GOAL_HEIGHT_INCHES - RobotConstants.Outtake.LAUNCH_HEIGHT_INCHES;

        // How far the piece must have fallen below its straight launch line by the time
        // it reaches the goal. Zero or negative means the launch line never gets that high.
        double fall = distanceInches * Math.tan(angle) - rise;
        if (distanceInches <= 0.0 || fall <= 0.0) {
            return Double.NaN;
        }
        return (distanceInches / Math.cos(angle)) * Math.sqrt(GRAVITY_INCHES_PER_SEC2 / (2.0 * fall));
    }
}
