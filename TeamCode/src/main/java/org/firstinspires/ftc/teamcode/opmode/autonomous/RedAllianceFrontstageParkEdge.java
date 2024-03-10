package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.autonomous.RedFrontstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.StartingLocation;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;

@Autonomous(name = "Red Alliance Frontstage Park Edge", group = "Autonomous Red", preselectTeleOp = "Primary TeleOp")
public class RedAllianceFrontstageParkEdge extends LinearOpMode implements AutonomousOpmode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the trajectory follower
        TrajectoryFollower follower = new TrajectoryFollower(hardwareMap, telemetry, DELAY_FRONTSTAGE);

        waitForStart();

        // Follow the trajectory
        TrajectoryGenerator trajectoryGenerator = new RedFrontstageTrajectoryGenerator();
        follower.followTrajectory(trajectoryGenerator, StartingLocation.FRONTSTAGE, ParkingLocation.EDGE);
    }
}
