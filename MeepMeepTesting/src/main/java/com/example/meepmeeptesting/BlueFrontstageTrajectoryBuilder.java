package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class BlueFrontstageTrajectoryBuilder implements AutonomousTrajectoryBuilder {

    public static final Pose2d STARTING_POSE = new Pose2d(-36, 60, Math.toRadians(-90));

    public static final Pose2d INNER_SPIKE_POSITION = new Pose2d(-35, 32, Math.toRadians(0));
    public static final Pose2d MIDDLE_SPIKE_POSITION = new Pose2d(-36, 34, Math.toRadians(-90));
    public static final Pose2d OUTER_SPIKE_POSITION = new Pose2d(-47, 40, Math.toRadians(-90));

    public static final int INNER_SPIKE_BASE_HEADING = 0;
    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;
    public static final int OUTER_SPIKE_BASE_HEADING = -90;

    public static final Pose2d UNDER_STAGE_TARGET_POSITION = new Pose2d(-36, 58, Math.toRadians(0));

    public static final Pose2d BACK_STAGE_PARKING_POSITION = new Pose2d(59, 59, Math.toRadians(0));

    @Override
    public TrajectorySequence getTrajectory(TeamElementLocation targetLocation, DriveShim drive) {

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(STARTING_POSE);

        // The first step is to drive the robot from the starting position to the correct spike mark.
        Pose2d spikePose;
        int spikeHeading = 0;
        if (targetLocation == TeamElementLocation.INNER) {
            spikePose = INNER_SPIKE_POSITION;
            spikeHeading = BlueFrontstageTrajectoryBuilder.INNER_SPIKE_BASE_HEADING;
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            spikePose = MIDDLE_SPIKE_POSITION;
            spikeHeading = BlueFrontstageTrajectoryBuilder.MIDDLE_SPIKE_BASE_HEADING;
        } else {
            spikePose = OUTER_SPIKE_POSITION;
            spikeHeading = BlueFrontstageTrajectoryBuilder.OUTER_SPIKE_BASE_HEADING;
        }

        builder.splineToLinearHeading(spikePose, Math.toRadians(spikeHeading));

        builder.setTangent(Math.toRadians(90));

        // Drive under the stage
        builder.splineToLinearHeading(UNDER_STAGE_TARGET_POSITION, Math.toRadians(0));

        // Set the tangent to 0 so that the robot begins it's path by driving towards the backdrop
        builder.setTangent(Math.toRadians(0));

        // Park the robot
        builder.splineToLinearHeading(BACK_STAGE_PARKING_POSITION, Math.toRadians(0));

        return builder.build();
    }
}
