package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controllers.Controller;
import org.firstinspires.ftc.teamcode.controllers.DroneLauncherController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controllers.PixelCollectorController;
import org.firstinspires.ftc.teamcode.sensors.Sensor;

import java.util.Arrays;
import java.util.List;

/**
 * TeleOp used for driving the robot.
 */
@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    // Controllers
    public MecanumDriveController mecanumDriveController;
    public LiftController liftController;
    public PixelCollectorController pixelCollectorController;
    public DroneLauncherController droneLauncherController;
    public List<Controller> controllers;

    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot robot = Robot.getInstance(hardwareMap, telemetry);

        // Instantiate all the controllers on the robot
        mecanumDriveController = new MecanumDriveController(robot.mecanumDrive, telemetry);
        liftController = new LiftController(robot.lift, telemetry);
        pixelCollectorController = new PixelCollectorController(robot.leftPixelCollector, robot.rightPixelCollector, telemetry);
        droneLauncherController = new DroneLauncherController(robot.droneLauncher, telemetry);
        controllers = Arrays.asList(mecanumDriveController, liftController, droneLauncherController, pixelCollectorController);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        liftController.start();
        telemetry.addLine("Robot Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Make a copy of the gamepads so all processing of the inputs is consistent
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Process controller inputs
        for (Controller controller : controllers) {
            controller.execute(currentGamepad1, currentGamepad2);
        }

        telemetry.update();
    }
}
