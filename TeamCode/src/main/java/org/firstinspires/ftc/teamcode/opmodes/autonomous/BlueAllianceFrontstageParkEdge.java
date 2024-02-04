package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.BlueFrontstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

@Autonomous(name="Blue Alliance Frontstage Park Edge", group="Autonomous Blue")
public class BlueAllianceFrontstageParkEdge extends LinearOpMode {

    public static final Pose2d STARTING_POSE = new Pose2d(-36, 63.5, Math.toRadians(-90));

    private Lift lift;
    private PixelCollector rightPixelCollector;
    private PixelCollector leftPixelCollector;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Create the vision sensor
        VisionSensor visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"));
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
        TrajectoryGenerator trajectoryGenerator = new BlueFrontstageTrajectoryGenerator(element);

        // Drive to the correct spike mark
        telemetry.addLine("Drive to spike mark");
        telemetry.update();
        Trajectory toSpikeMark = trajectoryGenerator.toSpikeMark(drive.trajectoryBuilder(STARTING_POSE));
        drive.followTrajectory(toSpikeMark);

        // Drop off the top pixel at the spike mark
        telemetry.addLine("Drop off purple pixel");
        telemetry.update();
        // TODO: un-comment out these two lines. Also, do we need a way to tell when the pixel has
        //       been deposited on the spike mark, and in the interim keep the pixel deposit
        //       continuing
//        rightPixelCollector.toggleState(true);
//        rightPixelCollector.update();

        // TODO: un-comment out these two lines. Also, do we need a way to tell when the pixel has
        //       been reset?
        // Turn off the pixel container
//        rightPixelCollector.toggleState(false);
//        rightPixelCollector.update();

        // Drive to the proper location (edge, middle, center) in front of the backdrop at which
        // the arm is rotated
        telemetry.addLine("Drive to front of backdrop");
        telemetry.update();
        Trajectory toArmLiftPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmLiftPosition);

        // Move the lift and arm to the scoring position
        telemetry.addLine("Raise arm");
        telemetry.update();
        // TODO: un-comment out these two lines. Also, do we need a way to tell when the lift has
        //       reached it's target position, and in the interim update the arm?
        // lift.setTarget(Lift.Position.ScoreLow);
        // lift.update();

        // Move to the backdrop and score the botton pixel
        telemetry.addLine("Drive to backdrop and score yellow pixel");
        telemetry.update();
        Trajectory toBackdropPosition = trajectoryGenerator.toBackdropPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toBackdropPosition);
        // TODO: un-comment out these two lines. Also, do we need a way to tell when the pixel has
        //       been deposited on the backdrop, and in the interim keep the pixel deposit
        //       continuing
//        leftPixelCollector.toggleState(true);

        // TODO: un-comment out these two lines. Also, do we need a way to tell when the pixel has
        //       been reset?
        // Turn off the pixel container
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
        // TODO: un-comment out these two lines. Also, do we need a way to tell when the pixel has
        //       been deposited on the spike mark, and in the interim keep the pixel deposit
        //       continuing
//        lift.setTarget(Lift.Position.Intake);
//        lift.update();

        // Drive to the parking spot
        telemetry.addLine("Drive to parking spot");
        telemetry.update();
        Trajectory toParkingSpot = trajectoryGenerator.toParkingSpotEdge(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toParkingSpot);
    }
}
