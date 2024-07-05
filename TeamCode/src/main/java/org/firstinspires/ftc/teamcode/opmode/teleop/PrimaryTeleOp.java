package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.controller.DroneLauncherController;
import org.firstinspires.ftc.teamcode.controller.LiftController;
import org.firstinspires.ftc.teamcode.controller.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controller.PixelCollectorController;

import java.util.Arrays;
import java.util.Collection;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private Robot robot;
    private DroneLauncherController droneLauncherController;
    private LiftController liftController;
    private MecanumDriveController mecanumDriveController;
    private PixelCollectorController pixelCollectorController;
    private Collection<Controller> controllers;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot.init(hardwareMap, telemetry);
        robot = Robot.getInstance(hardwareMap, telemetry);

        droneLauncherController = new DroneLauncherController(robot.droneLauncher, telemetry);
        liftController = new LiftController(robot.lift, telemetry);
        mecanumDriveController = new MecanumDriveController(robot.mecanumDrive, telemetry);
        pixelCollectorController = new PixelCollectorController(robot.leftPixelCollector, robot.rightPixelCollector, telemetry);
        controllers = Arrays.asList(droneLauncherController, liftController, mecanumDriveController, pixelCollectorController);


        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        liftController.start();
        telemetry.addLine("Robot Started");
    }

    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Process controller inputs
        for (Controller controller : controllers) {
            controller.execute(currentGamepad1, currentGamepad2);
        }

        telemetry.update();
    }
}
