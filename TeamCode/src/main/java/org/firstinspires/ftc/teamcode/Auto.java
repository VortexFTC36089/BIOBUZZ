package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


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
