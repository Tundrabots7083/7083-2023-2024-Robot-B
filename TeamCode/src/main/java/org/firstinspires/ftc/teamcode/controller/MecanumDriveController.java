package org.firstinspires.ftc.teamcode.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.controller.subsystem.MecanumDrive;

/**
 * A controller for a mecanum drive chassis.
 */
@Config
public class MecanumDriveController implements Controller {

    public static double MAX_TURNING_MULT = 0.75; //Max turning speed multiplier
    public static double MAX_DRIVE_MULT = 1; //Max drive speed multiplier
    public static double SLOW_TURNING_MULT = 0.6; //Slow turning speed multiplier
    public static double SLOW_DRIVE_MULT = 0.75; //Slow drive speed multiplier
    private final MecanumDrive mecanumDrive;
    private double driveGain;
    private double turnGain;

    private boolean bumperPressed = false;

    /**
     * Creates a controller for a mecanum drive chassis.
     *
     * @param mecanumDrive the mecanum drive to control.
     */
    public MecanumDriveController(MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        this.driveGain = MAX_DRIVE_MULT;
        this.turnGain = MAX_TURNING_MULT;
    }

    private void setGain(Gamepad gamepad) {
        if (!bumperPressed && gamepad.left_bumper) {
            driveGain = driveGain == MAX_DRIVE_MULT ? SLOW_DRIVE_MULT : MAX_DRIVE_MULT;
            turnGain = turnGain == MAX_TURNING_MULT ? SLOW_TURNING_MULT : MAX_TURNING_MULT;
        }
        bumperPressed = gamepad.left_bumper;
    }

    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        Telemetry telemetry = MyRobot.getInstance().telemetry;

        setGain(gamepad1);

        double x = gamepad1.left_stick_x * driveGain;
        double y = -gamepad1.left_stick_y * driveGain;
        double turn = -gamepad1.right_stick_x * turnGain;
        telemetry.addData("[DRIVE] X", x);
        telemetry.addData("[DRIVE] Y", y);
        telemetry.addData("[DRIVE] Turn", turn);

        mecanumDrive.drive(x, y, turn);
    }
}
