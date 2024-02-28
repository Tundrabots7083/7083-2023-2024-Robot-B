package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;
import org.firstinspires.ftc.teamcode.tests.PixelCollectorTest;

@TeleOp(name = "Right PixelCollector Test", group = "test")
public class TestRightPixelCollector extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = Robot.getInstance(hardwareMap, telemetry);
        PixelCollector pixelCollector = robot.rightPixelCollector;
        PixelCollectorTest test = new PixelCollectorTest(pixelCollector);

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            test.run(gamepad1, gamepad2);
        }
    }
}
