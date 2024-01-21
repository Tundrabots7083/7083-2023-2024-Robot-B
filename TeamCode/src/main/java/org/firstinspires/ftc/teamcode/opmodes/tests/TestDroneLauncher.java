package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.DroneLauncherController;

@TeleOp (name="Drone Launcher Test", group="test")
public class TestDroneLauncher extends OpMode {
    private DroneLauncherController droneLauncherController;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        droneLauncherController = new DroneLauncherController(hardwareMap, telemetry);
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        droneLauncherController.execute(gamepad1, gamepad2);
    }
}

