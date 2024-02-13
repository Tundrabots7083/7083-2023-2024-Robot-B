package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.BlueBackstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.autonomous.StartingLocation;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;

@Autonomous(name="Blue Alliance Backstage Park Center", group="Autonomous Blue")
public class BlueAllianceBackstageParkCenter extends LinearOpMode implements AutonomousOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the trajectory follower
        TrajectoryFollower follower = new TrajectoryFollower(hardwareMap, telemetry, DELAY_BACKSTAGE_IN_MILLIS);

        waitForStart();

        // Follow the trajectory
        TrajectoryGenerator trajectoryGenerator = new BlueBackstageTrajectoryGenerator();
        follower.followTrajectory(trajectoryGenerator, StartingLocation.BACKSTAGE, ParkingLocation.CENTER);
    }
}