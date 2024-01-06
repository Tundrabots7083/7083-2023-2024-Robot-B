package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Alliance Right", group ="Active")
public class RedAllianceRightParkRight extends LinearOpMode {
    private AutoMecanumDrive drive;
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup to send telemetry data to the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Create the auto mecanum drive
        drive = new AutoMecanumDrive(hardwareMap, telemetry);

        // Get the spike mark with the team prop
        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        sleep(3 * 1000);
        FirstVisionProcessor.Selected selection = visionProcessor.getSelection();
        telemetry.addData("Spike Mark", selection);
        // visionPortal.close(); // TODO: to use the vision portal elsewhere, then remove this line

        telemetry.addLine("Initialization complete");
        telemetry.update();

        waitForStart();

        switch (selection) {
            case LEFT:
                telemetry.addLine("Left Spike Mark");
                telemetry.update();
                break;
            case RIGHT:
                telemetry.addLine("Right Spike Mark");
                telemetry.update();
                break;
            case MIDDLE:
                telemetry.addLine("Right Spike Mark");
                telemetry.update();
                middleSpikeMark();
                break;
            case NONE:
                telemetry.addLine("Team Prop Not Detected");
                telemetry.update();
        }
    }

    /**
     * Drops the purple pixel off on the middle spike mark, maneuvers to the backdrop and scores
     * the yellow pixel in the middle, and then parks the robot to the right side of the backdrop.
     */
    private void middleSpikeMark() {
        // Set starting pose
        Pose2d startingPose = new Pose2d(12, -63.5, Math.toRadians(90));
        drive.setPoseEstimate(startingPose);

        // Move to center spike mark
        Trajectory traj1 = drive.trajectoryBuilder(startingPose)
                .forward(12)
                .build();

        // Move to center position on backdrop
        Trajectory traj2a = drive.trajectoryBuilder(traj1.end())
                .back(3)
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj2a.end())
                .back(22)
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj2b.end())
                .strafeRight(5)
                .build();

        // Park to the right of the backdrop
        Trajectory traj3a = drive.trajectoryBuilder(traj2c.end())
                .strafeLeft(28)
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj3a.end())
                .back(6)
                .build();

        // Unpack the robot
        telemetry.addLine("Drop Brush Roller");
        telemetry.addLine("Move Pixel Container to Intake Position");
        telemetry.update();

        // Drive to the spike mark and drop off the pixel
        drive.followTrajectory(traj1);
        telemetry.addLine("Drop Off Purple Pixel");
        telemetry.update();
        sleep(1000);

        // Drive to the backdrop, raise the arm and rotate the pixel container, and score the pixel
        drive.followTrajectory(traj2a);
        drive.turn(Math.toRadians(45));
        drive.followTrajectory(traj2b);
        drive.followTrajectory(traj2c);
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
    }
}