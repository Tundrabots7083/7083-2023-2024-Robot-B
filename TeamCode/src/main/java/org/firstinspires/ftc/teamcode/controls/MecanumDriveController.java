package org.firstinspires.ftc.teamcode.controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
public class MecanumDriveController implements Controller {

    public static double MAX_TURNING_MULT = 0.7; //Max turning speed multiplier
    public static double MAX_DRIVE_MULT = 1; //Max drive speed multiplier
    public static double SLOW_TURNING_MULT = 0.25; //Slow turning speed multiplier
    public static double SLOW_DRIVE_MULT = 0.3; //Slow drive speed multiplier
    private MecanumDrive mecanumDrive;
    private double driveGain;
    private double turnGain;

    private boolean bumperPressed = false;
    public MecanumDriveController(){
        this.driveGain = MAX_DRIVE_MULT;
        this.turnGain = MAX_TURNING_MULT;
    }

    @Override
    public void init(HardwareMap hwMap) {
        mecanumDrive = new MecanumDrive("driveTrain", "Drive Train");
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

        mecanumDrive.drive(x, y, turn);
    }
}
