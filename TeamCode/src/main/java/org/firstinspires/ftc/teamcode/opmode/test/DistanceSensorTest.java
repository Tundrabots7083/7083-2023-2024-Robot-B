package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DistanceSensor;

@TeleOp(name = "Distance Sensor Test", group = "test")
public class DistanceSensorTest extends OpMode {
    private DistanceSensor distanceSensor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        distanceSensor = new DistanceSensor(hardwareMap, telemetry, "leftDistanceSensor");

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        double distance = distanceSensor.getDistance();
        telemetry.addData("Distance", distance);
        telemetry.update();
    }
}
