package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Top-level robot object. OpModes construct one Robot and talk to subsystems.
 *
 * Adding a mechanism:
 *   1. Add hardware in {@link RobotHardware} and names in Constants.
 *   2. Create a subsystem class.
 *   3. Construct it here and expose it as a public final field.
 */
public class Robot {

    public final Drivetrain drivetrain;
    public final Intake intake;

    public Robot(HardwareMap hardwareMap) {
        RobotHardware hw = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hw);
        intake = new Intake(hw);
    }

    /** Stop every mechanism. Call at OpMode end and on emergency stop paths. */
    public void stopAll() {
        drivetrain.stop();
        intake.stop();
    }
}
