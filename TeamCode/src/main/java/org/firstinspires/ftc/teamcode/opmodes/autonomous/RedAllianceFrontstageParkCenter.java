package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.autonomous.RedFrontstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.StartingLocation;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;

@Autonomous(name="Red Alliance Frontstage Park Center", group="Autonomous Red")
public class RedAllianceFrontstageParkCenter extends LinearOpMode implements AutonomousOpmode{
    private static long DELAY_OPMODE_IN_MILLIS = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the trajectory follower
        TrajectoryFollower follower = new TrajectoryFollower(hardwareMap, telemetry, DELAY_FRONTSTAGE_IN_MILLIS);

        waitForStart();

        // Follow the trajectory
        TrajectoryGenerator trajectoryGenerator = new RedFrontstageTrajectoryGenerator();
        follower.followTrajectory(trajectoryGenerator, StartingLocation.FRONTSTAGE, ParkingLocation.CENTER);
    }
}
