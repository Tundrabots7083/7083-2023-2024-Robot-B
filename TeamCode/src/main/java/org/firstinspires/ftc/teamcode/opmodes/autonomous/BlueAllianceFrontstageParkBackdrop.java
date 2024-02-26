package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movement.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.autonomous.trajectories.BlueFrontstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.trajectories.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.field.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.field.RobotStartingLocation;

@Autonomous(name = "Blue Alliance Frontstage Park Backdrop", group = "Autonomous Blue", preselectTeleOp = "Primary TeleOp")
public class BlueAllianceFrontstageParkBackdrop extends LinearOpMode implements AutonomousOpmode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create the trajectory follower
        TrajectoryFollower follower = new TrajectoryFollower(hardwareMap, telemetry, DELAY_PARK_AT_BACKDROP);

        waitForStart();

        // Follow the trajectory
        TrajectoryGenerator trajectoryGenerator = new BlueFrontstageTrajectoryGenerator();
        follower.followTrajectory(trajectoryGenerator, RobotStartingLocation.FRONTSTAGE, RobotParkingLocation.IN_FRONT_OF_BACKDROP);
    }
}
