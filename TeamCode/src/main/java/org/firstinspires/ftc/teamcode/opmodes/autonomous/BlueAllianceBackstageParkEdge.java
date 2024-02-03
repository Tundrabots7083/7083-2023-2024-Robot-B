package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.BlueBackstageTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensors.VisionSensor;

@Autonomous(name="Blue Alliance Backstage Park Edge", group="Autonomous Blue")
public class BlueAllianceBackstageParkEdge extends LinearOpMode {

    public static final Pose2d STARTING_POSE = new Pose2d(12, 63.5, Math.toRadians(-90));

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

        // Create the pixel mover and arm
//        PixelMover pixelMover = new PixelMover("pixelMover", "Collects pixels and moves them", hardwareMap);
//        Arm arm = new Arm("arm", "Arm", hardwareMap);

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

        // Lower the pixel container to the intake and deposit location
        telemetry.addLine("Lower the pixel container");
//        arm.setTarget(Arm.Position.Start);
//        arm.update();
//        arm.setTarget(Arm.Position.Intake);
//        arm.update();
//        arm.stopArm();

        // Lock the pixels into the container
        telemetry.addLine("Lock the pixels");
        telemetry.update();
//        pixelMover.start(telemetry, true);

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
//        pixelMover.dropOffTopPixel(telemetry);

        // Drive to the proper location (edge, middle, center) in front of the backdrop at which
        // the arm is rotated
        telemetry.addLine("Drive to front of backdrop");
        telemetry.update();
        Trajectory toArmLiftPosition = trajectoryGenerator.toArmLiftPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmLiftPosition);

        // Move the arm to the scoring position
        telemetry.addLine("Raise arm");
        telemetry.update();
//        arm.setTarget(Arm.Position.ScoreLow);
//        while (!arm.isAtTarget()) {
//            arm.update();
//        }
//        arm.stopArm();

        // Move to the backdrop and score the botton pixel
        telemetry.addLine("Drive to backdrop and score yellow pixel");
        telemetry.update();
        Trajectory toBackdropPosition = trajectoryGenerator.toBackdropPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toBackdropPosition);
//        pixelMover.dropOffBottomPixel(telemetry);

        // Backup from the backdrop so the arm won't hit the backdrop when being lowered
        telemetry.addLine("Score yellow pixel on backdrop");
        telemetry.update();
        Trajectory toArmRetractionPosition = trajectoryGenerator.toArmRetractionPosition(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toArmRetractionPosition);

        // Lower the arm to the pixel intake position
        telemetry.addLine("Lower arm");
        telemetry.update();
//        arm.setTarget(Arm.Position.Intake);
//        while (!arm.isAtTarget()) {
//            arm.update();
//        }
//        arm.stopArm();

        // Drive to the edge parking spot
        telemetry.addLine("Drive to parking spot");
        telemetry.update();
        Trajectory toParkingSpot = trajectoryGenerator.toParkingSpotEdge(drive.trajectoryBuilder(drive.getPoseEstimate(), true));
        drive.followTrajectory(toParkingSpot);
    }
}
