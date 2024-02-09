package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.RedFrontstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

@Autonomous(name="Red Alliance Frontstage Park Edge", group="Autonomous Red")
public class RedAllianceFrontstageParkEdge extends LinearOpMode {
    public static long PIXEL_SPIKE_MARK_TIMER = 3250;
    public static long PIXEL_BACKDROP_TIMER = 500;
    public static long RAISE_LIFT_TIMER = 2000;
    public static long LOWER_LIFT_TIMER = 2000;

    public static final Pose2d STARTING_POSE = new Pose2d(-36, -63.125, Math.toRadians(90));

    private Lift lift;
    private PixelCollector rightPixelCollector;
    private PixelCollector leftPixelCollector;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();

        // Setup to send telemetry data to the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Create the vision sensor
        VisionSensor visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"), telemetry);
        visionSensor.initializeVisionPortal();

        // Create the drivetrain and set the initial pose
        AutoMecanumDrive drive = new AutoMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(STARTING_POSE);

        // Create the lift and pixel collectors
        lift = new Lift(hardwareMap, telemetry);
        rightPixelCollector = new PixelCollector("collectorRight", "Right pixel collector", hardwareMap, telemetry, false);
        leftPixelCollector = new PixelCollector("collectorLeft", "Left pixel collector", hardwareMap, telemetry, true);

        while(!visionSensor.webcamInitialized()) {}
        telemetry.addData("Webcam", "Initialized");
        telemetry.update();

        waitForStart();

        // Get the team element location and, once obtained, close the vision sensor as it is no
        // longer needed
        TeamElementLocation element = visionSensor.getTeamElementLocation();
        telemetry.addData("Element", element);
        telemetry.update();
        visionSensor.close();

        // Create the new trajectory generator
        TrajectoryGenerator trajectoryGenerator = new RedFrontstageTrajectoryGenerator(element);

        // Drive to the correct spike mark
        telemetry.addLine("Drive to spike mark");
        telemetry.update();
        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(drive.trajectoryBuilder(STARTING_POSE));
        drive.followTrajectory(toSpikeMark);

        // Drop off the top pixel at the spike mark
        telemetry.addLine("Drop off purple pixel");
        telemetry.update();
        leftPixelCollector.toggleState(true);
        leftPixelCollector.update();
        sleep(PIXEL_SPIKE_MARK_TIMER);

        // Turn off the pixel container
        leftPixelCollector.toggleState(false);
        leftPixelCollector.update();

        // Drive to the proper location (edge, middle, center) in front of the backdrop at which
        // the arm is rotated
        telemetry.addLine("Drive to front of backdrop");
        telemetry.update();
        Trajectory toArmLiftPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
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
        Trajectory toBackdropPosition = trajectoryGenerator.toBackdropPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toBackdropPosition);
        rightPixelCollector.toggleState(true);
        rightPixelCollector.update();
        timer.reset();
        while (timer.milliseconds() < PIXEL_BACKDROP_TIMER) {
            lift.update();
        }

        // Turn off the pixel container
        rightPixelCollector.toggleState(false);
        rightPixelCollector.update();

        // Backup from the backdrop so the arm won't hit the backdrop when being lowered
        telemetry.addLine("Score yellow pixel on backdrop");
        telemetry.update();
        Trajectory toArmRetractionPosition = trajectoryGenerator.toArmRetractionPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
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
        Trajectory toParkingSpot = trajectoryGenerator.toParkingSpotEdge(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toParkingSpot);
    }
}
