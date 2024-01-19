package org.firstinspires.ftc.teamcode.behaviortree;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.actions.DropPurplePixel;
import org.firstinspires.ftc.teamcode.behaviortree.actions.FollowTrajectory;
import org.firstinspires.ftc.teamcode.behaviortree.actions.IdentifySpikeMark;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

public class AutonomousBehaviorTreeNodes {

    public static final int RED_FRONTSTAGE_LEFT_SPIKE_X = -46;
    public static final int RED_FRONTSTAGE_LEFT_SPIKE_Y = -40;
    public static final int RED_FRONTSTAGE_LEFT_SPIKE_HEADING = 90;

    public static final int RED_FRONTSTAGE_RIGHT_SPIKE_X = -36;
    public static final int RED_FRONTSTAGE_RIGHT_SPIKE_Y = -34;
    public static final int RED_FRONTSTAGE_RIGHT_SPIKE_HEADING = 0;

    public static final int RED_FRONTSTAGE_MIDDLE_SPIKE_X = -40;
    public static final int RED_FRONTSTAGE_MIDDLE_SPIKE_Y = -40;
    public static final int RED_FRONTSTAGE_MIDDLE_SPIKE_HEADING = 90;

    public static final int RED_FRONTSTAGE_POST_SPIKE_X = -40;
    public static final int RED_FRONTSTAGE_POST_SPIKE_Y = -40;
    public static final int RED_FRONTSTAGE_POST_SPIKE_HEADING = 90;

    public static final int RED_PRE_BACKDROP_X = -34;
    public static final int RED_PRE_BACKDROP_Y = -58;
    public static final int RED_PRE_BACKDROP_HEADING = 0;



    private static Trajectory driveToSpikeMark(boolean isBlueTeam, boolean isBackstage, TeamElementLocation targetLocation, AutoMecanumDrive drive) {
        Trajectory driveToSpikeMark;

        double x = 0;
        double y = 0;
        double heading = 0;


        switch (targetLocation) {
            case LEFT:
                x = isBackstage ? RED_FRONTSTAGE_LEFT_SPIKE_X : RED_FRONTSTAGE_LEFT_SPIKE_X;
                y = isBackstage ? RED_FRONTSTAGE_LEFT_SPIKE_Y : RED_FRONTSTAGE_LEFT_SPIKE_Y;
                heading = isBackstage ? RED_FRONTSTAGE_LEFT_SPIKE_HEADING : RED_FRONTSTAGE_LEFT_SPIKE_HEADING;
                break;
            case RIGHT:
                x = isBackstage ? RED_FRONTSTAGE_RIGHT_SPIKE_X : RED_FRONTSTAGE_RIGHT_SPIKE_X;
                y = isBackstage ? RED_FRONTSTAGE_RIGHT_SPIKE_Y : RED_FRONTSTAGE_RIGHT_SPIKE_Y;
                heading = isBackstage ? RED_FRONTSTAGE_RIGHT_SPIKE_HEADING : RED_FRONTSTAGE_RIGHT_SPIKE_HEADING;
                break;
            case MIDDLE:
                x = isBackstage ? RED_FRONTSTAGE_MIDDLE_SPIKE_X : RED_FRONTSTAGE_MIDDLE_SPIKE_X;
                y = isBackstage ? RED_FRONTSTAGE_MIDDLE_SPIKE_Y : RED_FRONTSTAGE_MIDDLE_SPIKE_Y;
                heading = isBackstage ? RED_FRONTSTAGE_MIDDLE_SPIKE_HEADING : RED_FRONTSTAGE_MIDDLE_SPIKE_HEADING;
                break;
        }

        if (isBlueTeam) {
            x = -x;
            y = y;
            heading = -heading;
        }

        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(x, y), Math.toRadians(heading))
                .build();
    }

    private static Trajectory returnToStart(boolean isBlueTeam, boolean isBackstage, AutoMecanumDrive drive) {
        Trajectory returnToStart;

        double x = 0;
        double y = 0;
        double heading = 0;

        if (isBackstage) {
            x = isBlueTeam ? 0 : 0;
            y = isBlueTeam ? 0 : 0;
            heading = isBlueTeam ? 0 : 0;
        } else {
            x = isBlueTeam ? 0 : 0;
            y = isBlueTeam ? 0 : 0;
            heading = isBlueTeam ? 0 : 0;
        }

        if (isBlueTeam) {
            x = -x;
            y = y;
            heading = -heading;
        }

        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(x, y), Math.toRadians(heading))
                .build();
    }

    private static Trajectory frontStageToBackDrop(boolean isBlueTeam, boolean isBackstage, AutoMecanumDrive drive) {
        Trajectory frontStageToBackDrop;

        double x = 0;
        double y = 0;
        double heading = 0;





        if (isBlueTeam) {
            x = -x;
            y = y;
            heading = -heading;
        }

        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(x, y), Math.toRadians(heading))
                .build();
    }

    public static Node purplePixelSequence(boolean isBlueTeam, boolean isBackstage, TeamElementLocation targetLocation, Robot robot) {

        // Identify the spike mark
        Node identifySpikeMark = new IdentifySpikeMark("Identify Spike Mark", robot);
        // Drive to the spike mark
        Trajectory spikeMarkTrajectory = driveToSpikeMark(isBlueTeam, isBackstage, targetLocation, robot.drive);
        Node driveToSpikeMark = new FollowTrajectory("Drive to Spike Mark", robot, spikeMarkTrajectory);
        // Drop off the purple pixel
        Node dropPurplePixel = new DropPurplePixel("Drop Purple Pixel", robot);

        // Drive back to where the robot started
        Pose2d startingPosition = spikeMarkTrajectory.start();
        Trajectory driveBackToStart = robot.drive.trajectoryBuilder(spikeMarkTrajectory.end())
                .splineToLinearHeading(spikeMarkTrajectory.start(), 0)
                .build();

        // Drive to the backdrop
        if (isBackstage) {

        } else {
            // Drive to the backdrop
            // Drive to the middle spike mark
            // Drive to the backdrop
            // Drive to the middle spike mark
            // Drive to the backdrop
            // Drive to the middle spike mark
        }

        return null;

    }


}
