package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

/**
 * Generic code to follow a Roadrunner autonomous trajectory.
 */
public class TrajectoryFollower {
    public static long PIXEL_SPIKE_MARK_TIMER = 4000;
    public static long CONTAINER_CLOSED_TIMER = 500;
    public static long PIXEL_BACKDROP_TIMER = 2000;
    public static long RAISE_LIFT_TIMER = 2000;
    public static long LOWER_LIFT_TIMER = 2000;

    private final Telemetry telemetry;
    private final long delayOpmodeInMillis;

    private final Lift lift;
    private final AutoMecanumDrive drive;
    private final PixelCollector rightPixelCollector;
    private final PixelCollector leftPixelCollector;
    private final VisionSensor visionSensor;

    /**
     * Class to follow a given trajectory.
     * @param hardwareMap hardware map for the robot.
     * @param telemetry telemetry class for displaying data.
     * @param delayOpmodeInMillis number of milliseconds to delay the start of the autonomous
     *                            execution. This can be useful when starting in the frontstage to
     *                            give an alliance partner more time to complete their autonomous
     *                            run.
     */
    public TrajectoryFollower(HardwareMap hardwareMap, Telemetry telemetry, long delayOpmodeInMillis) {
        this.telemetry = telemetry;
        this.delayOpmodeInMillis = delayOpmodeInMillis;

        // Create the drivetrain and set the initial pose
        drive = new AutoMecanumDrive(hardwareMap, telemetry);

        // Create the lift and pixel collectors
        lift = new Lift(hardwareMap, telemetry);
        rightPixelCollector = new PixelCollector("collectorRight", "Right pixel collector", hardwareMap, telemetry, false);
        leftPixelCollector = new PixelCollector("collectorLeft", "Left pixel collector", hardwareMap, telemetry, true);

        // Create the vision sensor
        visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"), telemetry);
        visionSensor.initializeVisionPortal();
        while (!visionSensor.webcamInitialized()) {
        }
        telemetry.addLine("[VISION] Initialized");
    }

    /**
     * Wait for the specified number of milliseconds before returning.
     * @param milliseconds the number of milliseconds to wait.
     */
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Follow the given trajectory for the autonomous period.
     * @param trajectoryGenerator the trajectory generator for the autonomous period.
     * @param parkingLocation Where to park the robot.
     */
    public void followTrajectory(TrajectoryGenerator trajectoryGenerator, ParkingLocation parkingLocation) {
        ElapsedTime timer = new ElapsedTime();
        Pose2d startingPose = trajectoryGenerator.getStartingPose();

        // Get the team element location and, once obtained, close the vision sensor as it is no
        // longer needed
        TeamElementLocation element = visionSensor.getTeamElementLocation();
        telemetry.addData("Element", element);
        telemetry.update();
        visionSensor.close();

        sleep(delayOpmodeInMillis);

        // Set the starting pose
        drive.setPoseEstimate(startingPose);

        // Drive to the correct spike mark
        telemetry.addLine("Drive to spike mark");
        telemetry.update();
        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(drive.trajectoryBuilder(startingPose), element);
        drive.followTrajectory(toSpikeMark);

        // Drop off the purple pixel at the spike mark
        telemetry.addLine("Drop off purple pixel");
        telemetry.update();
        leftPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
        timer.reset();
        while (timer.milliseconds() < PIXEL_SPIKE_MARK_TIMER) {
            leftPixelCollector.update();
        }

        // Turn off the pixel container
        leftPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        timer.reset();
        while (timer.milliseconds() < CONTAINER_CLOSED_TIMER) {
            leftPixelCollector.update();
        }

        // Drive to the proper location (edge, middle, center) in front of the backdrop at which
        // the arm is rotated
        telemetry.addLine("Drive to front of backdrop");
        telemetry.update();
        Trajectory toArmLiftPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toArmLiftPosition);

        // Move the lift to the scoring position
        telemetry.addLine("Raise lift");
        telemetry.update();
        lift.setTarget(Lift.Position.Autonomous);
        timer.reset();
        while (timer.milliseconds() < RAISE_LIFT_TIMER) {
            lift.update();
        }

        // Move to the backdrop and score the yellow pixel
        telemetry.addLine("Drive to backdrop and score yellow pixel");
        telemetry.update();
        Trajectory toBackdropPosition = trajectoryGenerator.toBackdropPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toBackdropPosition);
        rightPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
        timer.reset();
        while (timer.milliseconds() < PIXEL_BACKDROP_TIMER) {
            rightPixelCollector.update();
            lift.update();
        }

        // Turn off the pixel container
        rightPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        timer.reset();
        while (timer.milliseconds() < CONTAINER_CLOSED_TIMER) {
            rightPixelCollector.update();
        }

        // Backup from the backdrop so the arm won't hit the backdrop when being lowered
        telemetry.addLine("Score yellow pixel on backdrop");
        telemetry.update();
        Trajectory toArmRetractionPosition = trajectoryGenerator.toArmRetractionPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toArmRetractionPosition);

        // Lower the lift and arm to the pixel intake position
        telemetry.addLine("Lower arm");
        telemetry.update();
        lift.setTarget(Lift.Position.Intake);
        timer.reset();
        while (timer.milliseconds() < LOWER_LIFT_TIMER) {
            lift.update();
        }

        // Drive to the edge parking spot
        telemetry.addLine("Drive to parking spot");
        telemetry.update();
        Trajectory toParkingSpot;
        if (parkingLocation == ParkingLocation.EDGE) {
            toParkingSpot = trajectoryGenerator.toParkingSpotEdge(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        } else {
            toParkingSpot = trajectoryGenerator.toParkingSpotCenter(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        }
        drive.followTrajectory(toParkingSpot);
    }
}
