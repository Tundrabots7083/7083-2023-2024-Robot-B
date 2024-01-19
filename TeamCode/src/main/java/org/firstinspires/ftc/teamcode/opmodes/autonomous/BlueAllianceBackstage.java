package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.BlueBackstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

public class BlueAllianceBackstage extends LinearOpMode {

    public static final Pose2d STARTING_POSE = new Pose2d(12, 63.5, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the robot
        Robot robot = new Robot(hardwareMap, telemetry);

        TrajectoryGenerator trajectoryGenerator = new BlueBackstageTrajectoryGenerator(TeamElementLocation.OUTER);

        robot.visionSensor.initializeVisionPortal();

        while(!robot.visionSensor.webcamInitialized()) {
            // Wait for webcam to initialize
            telemetry.addData("Webcam", "Initializing...");
            telemetry.update();
        }

        telemetry.addData("Webcam", "Initialized");
        telemetry.update();

        waitForStart();

        TeamElementLocation element = robot.visionSensor.getTeamElementLocation();

        telemetry.addData("Element", element);
        telemetry.update();

        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(robot.drive.trajectoryBuilder(STARTING_POSE));

        // Drive to the correct spike mark
        telemetry.addLine("Driving to spike mark");
        telemetry.update();
        robot.drive.followTrajectory(toSpikeMark);

        telemetry.addLine("Dropping off pixels");
        telemetry.update();
        // Deposit the purple pixel
        robot.pixelMoverController.dropOffPixels(); // TODO: Update when Ovies pushes changes to PixelMover

        sleep(3000);

        telemetry.addLine("Driving to parking spot");
        telemetry.update();
        // Drive to the parking spot
        Trajectory toParkingSpot = trajectoryGenerator.toParkingSpot(robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()));
        robot.drive.followTrajectory(toParkingSpot);

        while (opModeIsActive()) {
            // Do nothing
        }
    }
}
