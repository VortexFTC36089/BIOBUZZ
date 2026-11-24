package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Displays the REV Through Bore Encoder count while the turret is turned by hand.
 */
@TeleOp(name = "Test: REV Encoder Count", group = RobotConstants.OpModeGroups.TEST)
public class RevEncoderCountTest extends OpMode {

    private int startingCount;

    @Override
    public void init() {
        startingCount = TurretSubsystem.readEncoderTicks(hardwareMap);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Turn the turret by hand to watch the encoder count.");
        updateTelemetry();
    }

    @Override
    public void init_loop() {
        updateTelemetry();
    }

    @Override
    public void loop() {
        updateTelemetry();
    }

    private void updateTelemetry() {
        int currentCount = TurretSubsystem.readEncoderTicks(hardwareMap);
        telemetry.addData("Encoder port", RobotConstants.Turret.ENCODER_PORT);
        telemetry.addData("Encoder count", currentCount);
        telemetry.addData("Change from init", currentCount - startingCount);
        telemetry.update();
    }
}
