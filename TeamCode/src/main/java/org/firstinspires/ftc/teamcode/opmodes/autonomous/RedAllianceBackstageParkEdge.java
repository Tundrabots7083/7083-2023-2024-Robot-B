package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.autonomous.RedBackstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;

@Autonomous(name="Red Alliance Backstage Park Edge", group="Autonomous Red")
public class RedAllianceBackstageParkEdge extends LinearOpMode implements AutonomousOpmode{

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the trajectory follower
        TrajectoryFollower follower = new TrajectoryFollower(hardwareMap, telemetry, DELAY_BACKSTAGE_IN_MILLIS);

        waitForStart();

        // Follow the trajectory
        TrajectoryGenerator trajectoryGenerator = new RedBackstageTrajectoryGenerator();
        follower.followTrajectory(trajectoryGenerator, ParkingLocation.EDGE);
    }
}
