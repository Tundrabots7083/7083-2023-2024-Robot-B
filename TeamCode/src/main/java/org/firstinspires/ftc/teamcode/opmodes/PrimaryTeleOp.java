package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controls.Controller;
import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    private final static double CONTAINER_SERVO_POS = 0.62;

    private final Robot robot = new Robot();
    private Servo containerServo;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init(hardwareMap);
        containerServo = hardwareMap.get(Servo.class, "containerFlip");

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        containerServo.setPosition(CONTAINER_SERVO_POS);
        robot.pixelMoverController.start(telemetry);
        telemetry.addLine("Brush roller dropped.");
        telemetry.addData("[Container Servo] Pos", CONTAINER_SERVO_POS);
        telemetry.update();
    }

    @Override
    public void loop() {
        for (Controller controller : robot.controllers) {
            controller.execute(gamepad1, telemetry);
        }
    }
}
