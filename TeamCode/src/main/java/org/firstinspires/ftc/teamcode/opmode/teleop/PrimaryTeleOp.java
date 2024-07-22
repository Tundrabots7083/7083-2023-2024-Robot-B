package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.controller.DroneLauncherController;
import org.firstinspires.ftc.teamcode.controller.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controller.PixelCollectorController;
import org.firstinspires.ftc.teamcode.controller.ScoreSubsystemController;

import java.util.Arrays;
import java.util.Collection;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private Robot robot;
    private DroneLauncherController droneLauncherController;
    private ScoreSubsystemController liftController;
    private MecanumDriveController mecanumDriveController;
    private PixelCollectorController leftPixelCollectorController;
    private PixelCollectorController rightPixelCollectorController;
    private Collection<Controller> controllers;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);

        droneLauncherController = new DroneLauncherController(robot.droneLauncher);
        liftController = new ScoreSubsystemController(robot.scoringSubsystem);
        mecanumDriveController = new MecanumDriveController(robot.mecanumDrive);
        leftPixelCollectorController = new PixelCollectorController(robot.leftPixelCollector);
        rightPixelCollectorController = new PixelCollectorController(robot.rightPixelCollector);
        controllers = Arrays.asList(
                droneLauncherController,
                liftController,
                mecanumDriveController,
                leftPixelCollectorController,
                rightPixelCollectorController
        );

        telemetry.addLine("Initialization Complete");
        telemetry.update();
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
