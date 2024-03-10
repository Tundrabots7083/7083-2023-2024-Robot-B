package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.field.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.field.RobotStartingLocation;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.PixelCollector;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;

/**
 * Generic code to follow a Roadrunner autonomous trajectory.
 */
@Config
public class TrajectoryFollower {
    public static long PIXEL_SPIKE_MARK_TIMER = 2500;
    public static long COLLECTOR_CLOSED_TIMER = 500;
    public static long PIXEL_BACKDROP_TIMER = 1750;
    public static long LIFT_EXTRA_WAIT_TIME = 500;

    private final Telemetry telemetry;
    private final long delayOpModeInMillis;

    private final AutoMecanumDrive drive;

    private final Robot robot;

    /**
     * Class to follow a given trajectory.
     *
     * @param hardwareMap         hardware map for the robot.
     * @param telemetry           telemetry class for displaying data.
     * @param delayOpModeInMillis number of milliseconds to delay the start of the autonomous
     *                            execution. This can be useful when starting in the frontstage to
     *                            give an alliance partner more time to complete their autonomous
     *                            run.
     */
    public TrajectoryFollower(HardwareMap hardwareMap, Telemetry telemetry, long delayOpModeInMillis) {
        this.telemetry = telemetry;
        this.delayOpModeInMillis = delayOpModeInMillis;

        robot = Robot.getInstance(hardwareMap, telemetry, true);

        // Create the drivetrain and set the initial pose
        drive = new AutoMecanumDrive(hardwareMap, telemetry, robot.mecanumDrive);

        while (!robot.visionSensor.webcamInitialized()) {
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
        TeamElementLocation element = robot.visionSensor.getTeamElementLocation();
        telemetry.addData("Element", element);
        telemetry.update();
        //visionSensor.close();

        // Delay the start of the OpMode the requested number of milliseconds
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
        robot.leftPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
        timer.reset();
        while (timer.milliseconds() < PIXEL_SPIKE_MARK_TIMER) {
            robot.leftPixelCollector.update();
        }
        // Turn off the pixel collector
        telemetry.addLine("Turn off the pixel collector");
        telemetry.update();
        robot.leftPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        timer.reset();
        while (timer.milliseconds() < COLLECTOR_CLOSED_TIMER) {
            robot.leftPixelCollector.update();
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
            robot.lift.setTarget(Lift.Position.AUTONOMOUS_BACKSTAGE);
        } else {
            robot.lift.setTarget(Lift.Position.AUTONOMOUS_FRONTSTAGE);
        }
        while (!robot.lift.isAtTarget()) {
            robot.lift.update();
        }
        // Stop the lift from banging into the backdrop
        timer.reset();
        while (timer.milliseconds() < LIFT_EXTRA_WAIT_TIME) {
            robot.lift.update();
        }

        // Move to the backdrop
        telemetry.addLine("Drive the robot to the backdrop");
        telemetry.update();
        Trajectory toBackdropPosition = trajectoryGenerator.toBackdropPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toBackdropPosition);

        // Score the yellow pixel
        telemetry.addLine("Score the yellow pixel");
        telemetry.update();
        robot.rightPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
        timer.reset();
        while (timer.milliseconds() < PIXEL_BACKDROP_TIMER) {
            robot.rightPixelCollector.update();
            robot.lift.update();
        }

        // Turn off the pixel collector
        telemetry.addLine("Turn off the pixel collector");
        telemetry.update();
        robot.rightPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        timer.reset();
        while (timer.milliseconds() < COLLECTOR_CLOSED_TIMER) {
            robot.rightPixelCollector.update();
        }

        // Raise the lift if we are on the backstage, so that the black noodle doesn't de-score
        // the pixel
        if (startingLocation == RobotStartingLocation.BACKSTAGE) {
            telemetry.addLine("Raise the lift and arm out of the way");
            telemetry.update();
            robot.lift.setTarget(Lift.Position.AUTONOMOUS_FRONTSTAGE);
            while (!robot.lift.isAtTarget()) {
                robot.lift.update();
            }
            // Stop the lift from banging into the backdrop
            timer.reset();
            while (timer.milliseconds() < LIFT_EXTRA_WAIT_TIME) {
                robot.lift.update();
            }
        }

        // Backup from the backdrop so the arm won't hit the backdrop when being lowered
        telemetry.addLine("Drive the robot away from the backdrop");
        telemetry.update();
        Trajectory toArmRetractionPosition = trajectoryGenerator.toArmRetractionPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true), element);
        drive.followTrajectory(toArmRetractionPosition);

        // Lower the lift and arm to the pixel intake position
        telemetry.addLine("Lower the lift and arm");
        telemetry.update();
        robot.lift.setTarget(Lift.Position.INTAKE);
        while (!robot.lift.isAtTarget()) {
            robot.lift.update();
        }

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
