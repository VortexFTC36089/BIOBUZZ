package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A starter autonomous program. It uses time, not cameras or path following.
 *
 * Change the numbers, then test on the field.
 *   robot.drive(forward, strafe, turn)
 *   sleep(milliseconds)   1000 = 1 second
 *
 * Example:
 *   robot.drive(0.5, 0, 0);   // drive forward at half speed
 *   sleep(1000);              // for 1 second
 *   robot.stop();
 */
@Autonomous(name = "Simple Auto", group = "Competition")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("Simple Auto ready. Press START.");
        telemetry.update();
        waitForStart();

        // Drive forward for 1 second, then stop.
        // Edit these two numbers to change speed and time.
        robot.drive(0.5, 0, 0);
        sleep(1000);
        robot.stop();
    }
}
