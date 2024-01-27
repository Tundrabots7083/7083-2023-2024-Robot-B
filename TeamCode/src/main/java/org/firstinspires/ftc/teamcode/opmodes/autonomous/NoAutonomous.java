package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;

@Disabled
@Autonomous(name = "Front Stage (both alliances)", group ="Active")
public class NoAutonomous extends LinearOpMode {

    private PixelMover pixelMover;
    private Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        pixelMover = new PixelMover("pixelMover", "Pixel Mover", hardwareMap);
        lift = new Lift("arm", "Arm", hardwareMap);

        telemetry.addLine("Initialization Complete");

        waitForStart();

        lift.start();
        pixelMover.start(telemetry, true);
    }
}
