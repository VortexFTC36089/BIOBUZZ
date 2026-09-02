package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

/**
 * This is the program you run during the driver-controlled period.
 *
 * Controller (gamepad 1):
 *   Left stick up/down     drive forward / backward
 *   Left stick left/right  strafe left / right
 *   Right stick left/right turn
 *   Right bumper (hold)    slow mode
 *   Right trigger          intake in
 *   Left trigger           intake reverse
 *
 * Gamepad 2 can also use the triggers for intake.
 *
 * To change a button, edit this file. To change a motor speed or direction,
 * edit Robot.java instead.
 */

@TeleOp(name = "TeleOp", group = "Competition")
public class TeleOpMain extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    // Uncomment if slow mode is to be implemented
    // private boolean slowMode = false;
    // private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        
    }

    @Override
    public void loop() {

    }
}
