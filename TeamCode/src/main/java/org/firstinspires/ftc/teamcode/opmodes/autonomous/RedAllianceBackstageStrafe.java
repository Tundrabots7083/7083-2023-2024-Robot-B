package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;

@Disabled
@Config
@Autonomous(name = "Red Alliance Backstage Strafe", group ="Active")
public class RedAllianceBackstageStrafe extends LinearOpMode {
    public static double STRAFE_TIME = 3.5;
    public static double DRIVE_POWER = 0.5;

    private MecanumDrive drive;
    private PixelMover pixelMover;
    private Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Create the auto mecanum drive
        drive = new MecanumDrive("drive", "Mecanum Drive", hardwareMap);
        pixelMover = new PixelMover("pixelMover", "Pixel Mover", hardwareMap);
        lift = new Lift("arm", "Arm", hardwareMap);

        telemetry.addLine("Initialization Complete");

        waitForStart();

        lift.setTarget(Lift.Position.Intake);
        pixelMover.start(telemetry, false);

        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() < STRAFE_TIME) {
            drive.drive(DRIVE_POWER, 0, 0);
        }
    }
}
