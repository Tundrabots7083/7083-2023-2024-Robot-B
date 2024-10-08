package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.BlueBackstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.field.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.field.RobotStartingLocation;

@Autonomous(name = "Blue Alliance Backstage Park Center", group = "Autonomous Blue", preselectTeleOp = "Primary TeleOp")
public class BlueAllianceBackstageParkCenter extends LinearOpMode implements AutonomousOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the trajectory follower
        TrajectoryFollower follower = new TrajectoryFollower(hardwareMap, telemetry, DELAY_BACKSTAGE);

        waitForStart();

        // Follow the trajectory
        TrajectoryGenerator trajectoryGenerator = new BlueBackstageTrajectoryGenerator();
        follower.followTrajectory(trajectoryGenerator, RobotStartingLocation.BACKSTAGE, RobotParkingLocation.CENTER_OF_FIELD);
    }
}