package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

public class MecanumDriveCommand extends CommandBaseEx {
    public static double MAX_TURNING_MULT = 0.75; //Max turning speed multiplier
    public static double MAX_DRIVE_MULT = 1; //Max drive speed multiplier
    public static double SLOW_TURNING_MULT = 0.6; //Slow turning speed multiplier
    public static double SLOW_DRIVE_MULT = 0.75; //Slow drive speed multiplier

    private final MecanumDrive mecanumDrive;
    private final Gamepad gamepad1;

    private double driveGain = MAX_DRIVE_MULT;
    private double turnGain = MAX_TURNING_MULT;

    private boolean bumperPressed = false;

    public MecanumDriveCommand(MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2) {
        this.mecanumDrive = mecanumDrive;
        this.gamepad1 = gamepad1;

        addRequirements(mecanumDrive);
    }

    private void setGain(Gamepad gamepad) {
        if (!bumperPressed && gamepad.left_bumper) {
            driveGain = driveGain == MAX_DRIVE_MULT ? SLOW_DRIVE_MULT : MAX_DRIVE_MULT;
            turnGain = turnGain == MAX_TURNING_MULT ? SLOW_TURNING_MULT : MAX_TURNING_MULT;
        }
        bumperPressed = gamepad.left_bumper;
    }

    @Override
    public void execute() {
        Telemetry telemetry = MyRobot.getInstance().telemetry;

        setGain(gamepad1);

        double xValue = gamepad1.left_stick_x * driveGain;
        double yValue = -gamepad1.left_stick_y * driveGain;
        double turnValue = -gamepad1.right_stick_x * turnGain;

        mecanumDrive.drive(xValue, yValue, turnValue);

        telemetry.addData("[DRIVE] X", xValue);
        telemetry.addData("[DRIVE] Y", yValue);
        telemetry.addData("[DRIVE] Turn", turnValue);
    }
}
