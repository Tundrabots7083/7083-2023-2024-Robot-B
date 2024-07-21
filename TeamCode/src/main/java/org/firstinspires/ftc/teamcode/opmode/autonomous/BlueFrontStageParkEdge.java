package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MyRobot;

@Config
@Autonomous(name = "Blue Front Stage Park Edge", group = "Autonomous", preselectTeleOp = "Primary TeleOp")
public class BlueFrontStageParkEdge extends LinearOpMode {
    private MyRobot robot;

    /**
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = MyRobot.init(hardwareMap, telemetry, MyRobot.OpModeType.AUTO);

        while (!robot.webCam.isInitialized()) {
            // No-Op
        }
        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        // Do our thing....
    }
}

