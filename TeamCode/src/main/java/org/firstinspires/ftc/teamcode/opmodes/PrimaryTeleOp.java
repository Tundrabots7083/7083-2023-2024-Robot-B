package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controls.Controller;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
    private final static double CONTAINER_FLIP_SERVO_POS = 0.625;

    private final Robot robot = new Robot();
    private Servo containerFlipServo;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init(hardwareMap);
        containerFlipServo = hardwareMap.get(Servo.class, "containerFlip");

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        containerFlipServo.setPosition(CONTAINER_FLIP_SERVO_POS);
        telemetry.addData("[Container Servo] Pos", CONTAINER_FLIP_SERVO_POS);
        robot.pixelMoverController.start(telemetry);
        telemetry.addLine("Brush roller dropped.");
        telemetry.update();
    }

    @Override
    public void loop() {
        for (Controller controller : robot.controllers) {
            controller.execute(gamepad1, telemetry);
        }
    }
}
