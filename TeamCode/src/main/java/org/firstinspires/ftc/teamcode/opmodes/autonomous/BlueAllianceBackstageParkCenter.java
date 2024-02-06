package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.BlueBackstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

@Config
@Autonomous(name="Blue Alliance Backstage Park Center", group="Autonomous Blue")
public class BlueAllianceBackstageParkCenter extends LinearOpMode {
    public static long PIXEL_DROPOFF_TIMER = 3000;

    public static final Pose2d STARTING_POSE = new Pose2d(15.25, 63.125, Math.toRadians(-90));
    private Lift lift;
    private PixelCollector rightPixelCollector;
    private PixelCollector leftPixelCollector;

    @Override
    public void runOpMode() throws InterruptedException {
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

        // Wait for webcam to initialize
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
        TrajectoryGenerator trajectoryGenerator = new BlueBackstageTrajectoryGenerator(element);

        // Drive to the correct spike mark
        telemetry.addLine("Drive to spike mark");
        telemetry.update();
        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(drive.trajectoryBuilder(STARTING_POSE));
        drive.followTrajectory(toSpikeMark);

        // Drop off the top pixel at the spike mark
        telemetry.addLine("Drop off purple pixel");
        telemetry.update();
        // TODO: un-comment out these three lines.
        leftPixelCollector.toggleState(true);
        leftPixelCollector.update();
        sleep(PIXEL_DROPOFF_TIMER);

        /*

        // Turn off the pixel container
        // TODO: un-comment out these two lines.
//        rightPixelCollector.toggleState(false);
//        rightPixelCollector.update();

        // Drive to the proper location (edge, middle, center) in front of the backdrop at which
        // the arm is rotated
        telemetry.addLine("Drive to front of backdrop");
        telemetry.update();
        Trajectory toArmLiftPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmLiftPosition);

        // Move the arm to the scoring position
        telemetry.addLine("Raise arm");
        telemetry.update();
        // TODO: un-comment out these three lines.
//        lift.setTarget(Lift.Position.ScoreLow);
//        lift.update();
//        sleep(500); // TODO: see if this is long enough

        // Move to the backdrop and score the botton pixel
        telemetry.addLine("Drive to backdrop and score yellow pixel");
        telemetry.update();
        Trajectory toBackdropPosition = trajectoryGenerator.toBackdropPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toBackdropPosition);
        // TODO: un-comment out these two lines.
//        leftPixelCollector.toggleState(true);
//        sleep(250); // TODO: see if this is long enough

        // Turn off the pixel container
        // TODO: un-comment out these two lines.
//        leftPixelCollector.toggleState(false);
//        leftPixelCollector.update();

        // Backup from the backdrop so the arm won't hit the backdrop when being lowered
        telemetry.addLine("Score yellow pixel on backdrop");
        telemetry.update();
        Trajectory toArmRetractionPosition = trajectoryGenerator.toArmRetractionPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmRetractionPosition);

        // Lower the lift and arm to the pixel intake position
        telemetry.addLine("Lower arm");
        telemetry.update();
        // TODO: un-comment out these three lines.
//        lift.setTarget(Lift.Position.Intake);
//        lift.update();
//        sleep(250); // TODO: see if this is long enough

        // Drive to the center parking spot
        telemetry.addLine("Drive to parking spot");
        telemetry.update();
        Trajectory toParkingSpot = trajectoryGenerator.toParkingSpotCenter(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toParkingSpot);
         */
    }
}