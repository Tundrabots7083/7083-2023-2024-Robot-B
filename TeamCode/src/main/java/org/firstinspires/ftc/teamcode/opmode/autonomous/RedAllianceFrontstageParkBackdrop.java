package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.RedFrontstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.field.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.field.RobotStartingLocation;

@Autonomous(name = "Red Alliance Frontstage Park Backdrop", group = "Autonomous Red", preselectTeleOp = "Primary TeleOp")
public class RedAllianceFrontstageParkBackdrop extends LinearOpMode implements AutonomousOpmode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the trajectory follower
        TrajectoryFollower follower = new TrajectoryFollower(hardwareMap, telemetry, DELAY_PARK_AT_BACKDROP);

        waitForStart();

        // Follow the trajectory
        TrajectoryGenerator trajectoryGenerator = new RedFrontstageTrajectoryGenerator();
        follower.followTrajectory(trajectoryGenerator, RobotStartingLocation.FRONTSTAGE, RobotParkingLocation.IN_FRONT_OF_BACKDROP);
    }
}
