package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controllers.Controller;
import org.firstinspires.ftc.teamcode.sensors.Sensor;

/**
 * TeleOp used for driving the robot.
 */
@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = new Robot(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.liftController.start();
        telemetry.addLine("Robot Started");
    }

    @Override
    public void loop() {
        // Update sensors
        for (Sensor sensor : robot.sensors) {
            sensor.update();
        }

        // Process controller inputs
        for (Controller controller : robot.controllers) {
            controller.execute(gamepad1, gamepad2);
        }

        telemetry.update();
    }
}
