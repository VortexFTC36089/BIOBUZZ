package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Tests the outtake motor in isolation.
 *
 * <p>Hold right bumper to spin the outtake motor up to target RPM.
 */
@TeleOp(name = "Test: Outtake", group = RobotConstants.OpModeGroups.TEST)
public class OuttakeTest extends OpMode {

    private OuttakeSubsystem outtake;

    /**
     * Connects to the outtake motor.
     */
    @Override
    public void init() {
        outtake = new OuttakeSubsystem(hardwareMap);
        telemetry.addLine("RB = outtake motor  |  LT hold = open gate");
        telemetry.update();
    }

    /**
     * Applies bumper command and displays current outtake state.
     */
    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            outtake.outtake();
        } else {
            outtake.stopMotor();
        }
        outtake.setGateOpen(gamepad1.left_trigger > RobotConstants.Intake.TRIGGER_DEADZONE);

        outtake.update();
        telemetry.addData("Motor State", outtake.getMotorState());
        telemetry.addData("Left RPM", "%.0f / %.0f", outtake.getLeftRpm(), outtake.getTargetRpm());
        telemetry.addData("Right RPM", "%.0f / %.0f", outtake.getRightRpm(), outtake.getTargetRpm());
        telemetry.addData("At Speed", outtake.atSpeed());
        telemetry.addData("Gate", outtake.isGateOpen() ? "OPEN" : "CLOSED");
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
