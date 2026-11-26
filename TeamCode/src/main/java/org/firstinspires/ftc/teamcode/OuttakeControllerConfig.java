package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Live tuning values for {@link OuttakeSubsystem}, exposed through the
 * project's ByLazar configurable panels.
 *
 * <p>All gains are deliberately zero placeholders until measured on the robot.
 * A wheel whose gains are still zero receives zero voltage, while the other
 * wheel can be tuned independently in {@link OuttakeTuning}.
 */
@Configurable
public final class OuttakeControllerConfig {

    /** Static feedforward voltage, in volts. */
    public static double LEFT_KS_VOLTS = .89;
    /** Feedforward voltage per RPM. */
    public static double LEFT_KV_VOLTS_PER_RPM = 0.001949;
    /** Feedback voltage per RPM of velocity error. */
    public static double LEFT_KP_VOLTS_PER_RPM = 0.031;

    /** Static feedforward voltage, in volts. */
    public static double RIGHT_KS_VOLTS = .84;
    /** Feedforward voltage per RPM. */
    public static double RIGHT_KV_VOLTS_PER_RPM = 0.001971;
    /** Feedback voltage per RPM of velocity error. */
    public static double RIGHT_KP_VOLTS_PER_RPM = 0.031;

    /** Maximum normalized motor power in [0, 1]. */
    public static double MAX_OUTPUT = 1.0;

    /**
     * Exponential velocity-filter weight in (0, 1]. 1.0 disables filtering;
     * lower values smooth encoder noise at the cost of added feedback latency.
     */
    public static double VELOCITY_FILTER_ALPHA = 1.0;

    private OuttakeControllerConfig() {
        // Utility class.
    }
}
