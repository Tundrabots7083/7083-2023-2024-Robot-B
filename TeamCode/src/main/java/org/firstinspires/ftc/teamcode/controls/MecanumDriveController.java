package org.firstinspires.ftc.teamcode.controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.dataflow.qual.TerminatesExecution;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Config
public class MecanumDriveController implements Controller {

    public static double MAX_TURNING_MULT = 0.7; //Max turning speed multiplier
    public static double MAX_DRIVE_MULT = 1; //Max drive speed multiplier
    public static double SLOW_TURNING_MULT = 0.25; //Slow turning speed multiplier
    public static double SLOW_DRIVE_MULT = 0.3; //Slow drive speed multiplier
    private final MecanumDrive mecanumDrive;
    private final Telemetry telemetry;
    private double driveGain;
    private double turnGain;

    private boolean bumperPressed = false;
    public MecanumDriveController(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.driveGain = MAX_DRIVE_MULT;
        this.turnGain = MAX_TURNING_MULT;
        mecanumDrive = new MecanumDrive("driveTrain", "Drive Train", hardwareMap);
    }

    private void setGain(Gamepad gamepad) {
        if (!bumperPressed && (gamepad.right_bumper || gamepad.left_bumper)) {
            driveGain = driveGain == MAX_DRIVE_MULT ? SLOW_DRIVE_MULT : MAX_DRIVE_MULT;
            turnGain = turnGain == MAX_TURNING_MULT ? SLOW_TURNING_MULT : MAX_TURNING_MULT;
        }
        bumperPressed = gamepad.right_bumper || gamepad.left_bumper;
    }
    public void execute(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
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
