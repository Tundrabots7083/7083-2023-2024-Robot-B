package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class MecanumDriveController {

    private static final double MAX_TURNING_MULT = 0.7; //Max turning speed multiplier
    private static final double MAX_DRIVE_MULT = 1; //Max drive speed multiplier
    private static final double SLOW_TURNING_MULT = 0.25; //Slow turning speed multiplier
    private static final double SLOW_DRIVE_MULT = 0.3; //Slow drive speed multiplier
    private final Robot robot;
    private double driveGain;
    private double turnGain;

    private boolean bumperPressed = false;
    public MecanumDriveController(Robot robot){
        this.robot = robot;
        this.driveGain = MAX_DRIVE_MULT;
        this.turnGain = MAX_TURNING_MULT;
    }

    private void setGain(Gamepad gamepad) {
        if (!bumperPressed && (gamepad.right_bumper || gamepad.left_bumper)) {
            driveGain = driveGain == MAX_DRIVE_MULT ? SLOW_DRIVE_MULT : MAX_DRIVE_MULT;
            turnGain = turnGain == MAX_TURNING_MULT ? SLOW_TURNING_MULT : MAX_TURNING_MULT;
        }
        bumperPressed = gamepad.right_bumper || gamepad.left_bumper;
    }
    public void execute(Gamepad gamepad, Telemetry telemetry) {
        setGain(gamepad);

        double x = gamepad.left_stick_x * driveGain;
        double y = -gamepad.left_stick_y * driveGain;
        double turn = gamepad.right_stick_x * turnGain;

        robot.mecanumDrive.drive(x, y, turn);
    }
}
