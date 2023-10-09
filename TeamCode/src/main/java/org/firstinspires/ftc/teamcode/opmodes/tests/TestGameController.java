package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test Game Controller", group = "test")
public class TestGameController extends OpMode {

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);

        if (gamepad1.a) {
            telemetry.addLine("A pressed");
        }
        if (gamepad1.b) {
            telemetry.addLine("B pressed");
        }
        if (gamepad1.x) {
            telemetry.addLine("X pressed");
        }
        if (gamepad1.y) {
            telemetry.addLine("Y pressed");
        }
        if (gamepad1.back) {
            telemetry.addLine("Back Pressed");
        }
        if (gamepad1.dpad_left) {
            telemetry.addLine("DPAD Left");
        }
        if (gamepad1.dpad_right) {
            telemetry.addLine("DPAD Right");
        }
        if (gamepad1.dpad_up) {
            telemetry.addLine("DPAD Up");
        }
        if (gamepad1.dpad_down) {
            telemetry.addLine("DPAD Down");
        }
        if (gamepad1.right_bumper) {
            telemetry.addLine("Right Bumper");
        }
        if (gamepad1.left_bumper) {
            telemetry.addLine("Left Bumper");
        }
        if (gamepad1.guide) {
            telemetry.addLine("Guide");
        }
        if (gamepad1.start) {
            telemetry.addLine("Start");
        }
        if (gamepad1.left_stick_button) {
            telemetry.addLine("Left Stick Button");
        }
        if (gamepad1.right_stick_button) {
            telemetry.addLine("Right Stick Button");
        }
    }
}