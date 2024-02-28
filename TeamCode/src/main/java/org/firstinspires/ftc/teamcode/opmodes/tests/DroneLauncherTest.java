package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DroneLauncher;

@TeleOp(name = "Drone Launcher Test", group = "test")
public class DroneLauncherTest extends LinearOpMode {
    public DroneLauncher droneLauncher;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        droneLauncher = new DroneLauncher(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        droneLauncher.setToLaunchAngle();
        sleep(250);
        droneLauncher.launchDrone();
    }
}
