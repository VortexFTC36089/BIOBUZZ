package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tests the outtake motor and servo gate in isolation.
 *
 * <p>Hold right bumper to spin the outtake motor up to target RPM, and press B
 * to toggle the servo gate between closed and open.
 */
@TeleOp(name = "Test: Outtake", group = RobotConstants.OpModeGroups.TEST)
public class OuttakeTest extends OpMode {

    private OuttakeSubsystem outtake;
    private final ToggleButton gateToggle = new ToggleButton();

    /**
     * Connects to the outtake motor and gate servo.
     */
    @Override
    public void init() {
        outtake = new OuttakeSubsystem(hardwareMap);
        telemetry.addLine("RB = outtake motor, B = toggle gate");
        telemetry.update();
    }

    /**
     * Applies bumper and button commands and displays current outtake state.
     */
    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            outtake.outtake();
        } else {
            outtake.stopMotor();
        }

        if (gateToggle.update(gamepad1.b)) {
            outtake.toggleGate();
        }

        outtake.update();
        telemetry.addData("Motor State", outtake.getMotorState());
        telemetry.addData("Outtake RPM", "%.0f / %.0f", outtake.getCurrentRpm(), outtake.getTargetRpm());
        telemetry.addData("At Speed", outtake.atSpeed());
        telemetry.addData("Gate State", outtake.getGateState());
        telemetry.update();
    }

    /**
     * Ensures the outtake motor stops when the OpMode ends.
     */
    @Override
    public void stop() {
        if (outtake != null) {
            outtake.stop();
            outtake.update();
        }
    }
}
