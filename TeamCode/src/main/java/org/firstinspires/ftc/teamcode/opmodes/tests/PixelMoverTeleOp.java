package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TestRobot;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;
@TeleOp (name="Pixel Mover TeleOp", group="test")
public class PixelMoverTeleOp extends OpMode {
    private final Robot robot = new TestRobot();
    private final PixelMoverController pixelMoverController = new PixelMoverController(robot);

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.pixelMover.init(hardwareMap);
        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        pixelMoverController.toggleState(gamepad1, telemetry);
    }
}
