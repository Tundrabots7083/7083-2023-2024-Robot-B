package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.controls.PixelMoverController;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;

@TeleOp (name="Pixel Mover TeleOp", group="test")
public class PixelMoverTeleOp extends OpMode {
    private PixelMover pixelMover;
    private final PixelMoverController pixelMoverController = new PixelMoverController();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pixelMover.init(hardwareMap);
        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        pixelMoverController.toggleState(gamepad1, telemetry);
    }
}
