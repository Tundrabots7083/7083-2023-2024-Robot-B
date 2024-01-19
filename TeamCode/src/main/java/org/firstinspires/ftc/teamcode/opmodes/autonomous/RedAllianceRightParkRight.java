package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Alliance Right", group ="Active")
public class RedAllianceRightParkRight extends LinearOpMode {
    private AutoMecanumDrive drive;
    private PixelMover pixelMover;
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Create the auto mecanum drive
        drive = new AutoMecanumDrive(hardwareMap, telemetry);

        // Create the pixel mover
        pixelMover = new PixelMover("pixelmover", "Pixel Mover", hardwareMap);

        // Get the spike mark with the team prop
        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam Front"), visionProcessor);
        sleep(3 * 1000);

        telemetry.addLine("Initialization complete");
        telemetry.update();

        waitForStart();

        // Find the randomized team prop. This must be after waitForStart() to allow for
        // randomization
        TeamElementLocation selection = visionProcessor.getSelection();
        telemetry.addData("Spike Mark", selection);
        visionPortal.close(); // TODO: to use the vision portal elsewhere, then remove this line

        switch (selection) {
            case INNER:
                telemetry.addLine("Left Spike Mark");
                telemetry.update();
                break;
            case OUTER:
                telemetry.addLine("Right Spike Mark");
                telemetry.update();
                break;
            case MIDDLE:
                telemetry.addLine("Right Spike Mark");
                telemetry.update();
                // middleSpikeMark();
                break;
            case NONE:
                telemetry.addLine("Team Prop Not Detected");
                telemetry.update();
        }
        middleSpikeMark();
    }

    /**
     * Drops the purple pixel off on the middle spike mark, maneuvers to the backdrop and scores
     * the yellow pixel in the middle, and then parks the robot to the right side of the backdrop.
     */
    private void middleSpikeMark() {
        // Set starting pose
        Pose2d startingPose = new Pose2d(12, -63.5, Math.toRadians(270));
        drive.setPoseEstimate(startingPose);

        // Move to center spike mark
        Trajectory traj1 = drive.trajectoryBuilder(startingPose)
                .forward(21.5)
                .build();

        // Move to center position on backdrop
        Trajectory traj2a = drive.trajectoryBuilder(traj1.end())
                .back(5)
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj2a.end())
                .strafeRight(20)
                .splineTo(new Vector2d(48, -35), Math.toRadians(90))
                .build();

        // Park to the right of the backdrop
        Trajectory traj3a = drive.trajectoryBuilder(traj2b.end())
                .strafeLeft(24)
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj3a.end())
                .back(10)
                .build();

        // Unpack the robot
        telemetry.addLine("Drop Brush Roller");
        telemetry.addLine("Move Pixel Container to Intake Position");
        telemetry.update();

        // Drive to the spike mark and drop off the pixel
        drive.followTrajectory(traj1);
        telemetry.addLine("Drop Off Purple Pixel");
        telemetry.update();
        pixelMover.dropOffPixels();
        sleep(1000);

        /*
        // Drive to the backdrop, raise the arm and rotate the pixel container, and score the pixel
        drive.followTrajectory(traj2a);
        drive.followTrajectory(traj2b);
        telemetry.addLine("Raise Arm and Rotate Pixel Container");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("Score Yellow Pixel");
        telemetry.update();
        sleep(1000);

        // Park the robot on the right side of the backdrop
        telemetry.addLine("Return Arm/Container to Intake Position");
        telemetry.update();
        sleep(1*1000);
        drive.followTrajectory(traj3a);
        drive.followTrajectory(traj3b);
        telemetry.addLine("Park Right");
        telemetry.update();
        */
    }
}