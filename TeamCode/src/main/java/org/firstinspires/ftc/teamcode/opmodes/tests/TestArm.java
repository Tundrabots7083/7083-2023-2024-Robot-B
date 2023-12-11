package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.ArmController;

@TeleOp (name="Arm Test", group="test")
public class TestArm extends OpMode {
    private final ArmController armController = new ArmController();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armController.init(hardwareMap);
        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        armController.execute(gamepad1, telemetry);
    }
}
