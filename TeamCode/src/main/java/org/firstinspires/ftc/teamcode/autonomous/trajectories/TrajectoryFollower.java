package org.firstinspires.ftc.teamcode.autonomous.trajectories;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.trajectories.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.field.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.field.RobotStartingLocation;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

/**
 * Generic code to follow a Roadrunner autonomous trajectory.
 */
@Config
public class TrajectoryFollower {
    public static long PIXEL_SPIKE_MARK_TIMER = 2500;
    public static long PIXEL_BACKDROP_TIMER = 1750;

    private final Telemetry telemetry;
    private final long delayOpModeInMillis;

    private final AutoMecanumDrive drive;
    private final Lift lift;
    private final PixelCollector rightPixelCollector;
    private final PixelCollector leftPixelCollector;
    private final VisionSensor visionSensor;

    /**
     * Class to follow a given trajectory.
     *
     * @param hardwareMap         hardware map for the robot.
     * @param telemetry           telemetry class for displaying data.
     * @param delayOpmodeInMillis number of milliseconds to delay the start of the autonomous
     *                            execution. This can be useful when starting in the frontstage to
     *                            give an alliance partner more time to complete their autonomous
     *                            run.
     */
    public TrajectoryFollower(HardwareMap hardwareMap, Telemetry telemetry, long delayOpmodeInMillis) {
        this.telemetry = telemetry;
        this.delayOpModeInMillis = delayOpmodeInMillis;

        // Create the drivetrain
        drive = new AutoMecanumDrive(hardwareMap, telemetry);

        // Get the lift and pixel collectors
        Robot robot = Robot.getInstance(hardwareMap, telemetry);
        lift = robot.lift;
        leftPixelCollector = robot.leftPixelCollector;
        rightPixelCollector = robot.rightPixelCollector;

        // Create the vision sensor
        visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"), telemetry);
        while (!visionSensor.webcamInitialized()) {
            // NO-OP
        }
        telemetry.addLine("[VISION] Initialized");
    }

    /**
     * Wait for the specified number of milliseconds before returning.
     *
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
     *
     * @param trajectoryGenerator the trajectory generator for the autonomous period.
     * @param parkingLocation     Where to park the robot.
     */
    public void followTrajectory(TrajectoryGenerator trajectoryGenerator, RobotStartingLocation startingLocation, RobotParkingLocation parkingLocation) {
        ElapsedTime timer = new ElapsedTime();
        Pose2d startingPose = trajectoryGenerator.getStartingPose();

        // Get the team element location and, once obtained, close the vision sensor as it is no
        // longer needed
        TeamElementLocation element = visionSensor.getTeamElementLocation();
        telemetry.addData("Element", element);
        telemetry.update();
        //visionSensor.close();

        // Delay the start of the opmode the requested number of milliseconds
        sleep(delayOpModeInMillis);

        // Set the starting pose
        drive.setPoseEstimate(startingPose);

        // Drive to the correct spike mark
        telemetry.addLine("Drive to the spike mark");
        telemetry.update();
        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(drive.trajectoryBuilder(startingPose), element);
        drive.followTrajectory(toSpikeMark);

        // Drop off the purple pixel at the spike mark
        telemetry.addLine("Drop off the purple pixel");
        telemetry.update();
        leftPixelCollector.setState(PixelCollector.State.DEPOSITING);
        timer.reset();
        while (timer.milliseconds() < PIXEL_SPIKE_MARK_TIMER) {
            leftPixelCollector.update();
        }
        // Turn off the pixel collector
        telemetry.addLine("Turn off the pixel collector");
        telemetry.update();
        leftPixelCollector.setState(PixelCollector.State.CLOSED);
        while (!leftPixelCollector.isStopped()) {
            leftPixelCollector.update();
        }

        // Drive to the proper location (edge, middle, center) in front of the backdrop at which
        // the arm is rotated
        telemetry.addLine("Drive to the front of the backdrop");
        telemetry.update();
        Trajectory toArmLiftPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toArmLiftPosition);

        // Move the lift to the scoring position
        telemetry.addLine("Raise the lift and arm");
        telemetry.update();
        if (startingLocation == RobotStartingLocation.BACKSTAGE) {
            lift.setTarget(Lift.Position.AUTONOMOUS_BACKSTAGE);
        } else {
            lift.setTarget(Lift.Position.AUTONOMOUS_FRONTSTAGE);
        }
        while (!lift.isAtTarget()) {
            lift.update();
        }
        lift.update();

        // Move to the backdrop
        telemetry.addLine("Drive the robot to the backdrop");
        telemetry.update();
        Trajectory toBackdropPosition = trajectoryGenerator.toBackdropPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toBackdropPosition);

        // Score the yellow pixel
        telemetry.addLine("Score the yellow pixel");
        telemetry.update();
        rightPixelCollector.setState(PixelCollector.State.DEPOSITING);
        timer.reset();
        while (timer.milliseconds() < PIXEL_BACKDROP_TIMER) {
            rightPixelCollector.update();
        }

        // Turn off the pixel collector
        telemetry.addLine("Turn off the pixel collector");
        telemetry.update();
        rightPixelCollector.setState(PixelCollector.State.CLOSED);
        while (!rightPixelCollector.isStopped()) {
            rightPixelCollector.update();
        }

        // Raise the lift if we are on the backstage, so that the black noodle doesn't descore
        // the pixel
        if (startingLocation == RobotStartingLocation.BACKSTAGE) {
            telemetry.addLine("Raise the lift and arm out of the way");
            telemetry.update();
            lift.setTarget(Lift.Position.AUTONOMOUS_FRONTSTAGE);
            while (!lift.isAtTarget()) {
                lift.update();
            }
            lift.update();
        }

        // Backup from the backdrop so the arm won't hit the backdrop when being lowered
        telemetry.addLine("Drive the robot away from the backdrop");
        telemetry.update();
        Trajectory toArmRetractionPosition = trajectoryGenerator.toArmRetractionPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toArmRetractionPosition);

        // Lower the lift and arm to the pixel intake position
        telemetry.addLine("Lower the lift and arm");
        telemetry.update();
        lift.setTarget(Lift.Position.INTAKE);
        while (!lift.isAtTarget()) {
            lift.update();
        }
        lift.update();

        // If not parking in front of the backdrop, drive to the parking location
        if (parkingLocation == RobotParkingLocation.IN_FRONT_OF_BACKDROP) {
            telemetry.addLine("Park in front of the backdrop");
        } else {
            // Drive to the parking spot
            telemetry.addLine("Drive to the parking spot");
            telemetry.update();
            Trajectory toParkingSpot;
            if (parkingLocation == RobotParkingLocation.EDGE_OF_FIELD) {
                toParkingSpot = trajectoryGenerator.toParkingSpotEdge(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
            } else {
                toParkingSpot = trajectoryGenerator.toParkingSpotCenter(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
            }
            drive.followTrajectory(toParkingSpot);
        }
    }
}
