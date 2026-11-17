package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

import java.util.Map;

/**
 * Shared keys for the FTC SDK blackboard used to pass state between OpModes.
 */
public final class RobotBlackboard {

    public static final String LAST_AUTO_POSE = "org.firstinspires.ftc.teamcode.lastAutoPose";
    public static final String LAST_AUTO_TURRET_ZERO_TICKS =
            "org.firstinspires.ftc.teamcode.lastAutoTurretZeroTicks";
    public static final String LAST_TELEOP_POSE = "org.firstinspires.ftc.teamcode.lastTeleOpPose";
    public static final String LAST_TELEOP_TURRET_ZERO_TICKS =
            "org.firstinspires.ftc.teamcode.lastTeleOpTurretZeroTicks";

    private RobotBlackboard() {
        // Constants and typed accessors only.
    }

    public static Pose getPose(Map<String, Object> blackboard, String key) {
        return getValue(blackboard, key, Pose.class);
    }

    public static Integer getInteger(Map<String, Object> blackboard, String key) {
        return getValue(blackboard, key, Integer.class);
    }

    private static <T> T getValue(Map<String, Object> blackboard, String key, Class<T> type) {
        Object value = blackboard.get(key);
        if (value == null) {
            return null;
        }
        if (!type.isInstance(value)) {
            throw new IllegalStateException(
                    "Blackboard value for " + key + " must be " + type.getSimpleName()
                            + ", but was " + value.getClass().getSimpleName()
            );
        }
        return type.cast(value);
    }
}
