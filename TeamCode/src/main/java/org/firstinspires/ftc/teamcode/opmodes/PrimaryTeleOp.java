package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controls.Controller;
import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {

    private final Robot robot = new Robot();

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        for (Controller controller : robot.controllers) {
            controller.execute(gamepad1, telemetry);
        }
    }
}
