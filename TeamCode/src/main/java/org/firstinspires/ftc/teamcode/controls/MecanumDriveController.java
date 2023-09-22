package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.inspection.GamepadInspection;

public class MecanumDriveController {
    private final Robot robot;
    public MecanumDriveController(Robot robot){
        this.robot = robot;
    }
    public void execute(Gamepad gamepad, Telemetry telemetry){
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;

        robot.mecanumDrive.drive(x, y, turn);
    }
}
