package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class BlueBackstageTrajectoryBuilder implements AutonomousTrajectoryBuilder {

    public static final Pose2d STARTING_POSE = new Pose2d(12, 63.5, Math.toRadians(-90));

    public static final Pose2d BACK_STAGE_INNER_SPIKE_BASE = new Pose2d(11, 32, Math.toRadians(180));
    public static final Pose2d BACK_STAGE_MIDDLE_SPIKE_BASE = new Pose2d(12, 34, Math.toRadians(-90));
    public static final Pose2d BACK_STAGE_OUTER_SPIKE_BASE = new Pose2d(23, 40, Math.toRadians(-90));

    public static final int INNER_SPIKE_BASE_HEADING = 180;
    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;
    public static final int OUTER_SPIKE_BASE_HEADING = -90;


    public static final Pose2d BACK_STAGE_PARKING_POSITION = new Pose2d(59, 59, Math.toRadians(180));

    @Override
    public TrajectorySequence getTrajectory(TeamElementLocation targetLocation, DriveShim drive) {

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(STARTING_POSE);

        // The first step is to drive the robot from the starting position to the correct spike mark.
        Pose2d spikePose;
        int spikeHeading = 0;
        if (targetLocation == TeamElementLocation.INNER) {
            spikePose = BACK_STAGE_INNER_SPIKE_BASE;
            spikeHeading = BlueBackstageTrajectoryBuilder.INNER_SPIKE_BASE_HEADING;
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            spikePose = BACK_STAGE_MIDDLE_SPIKE_BASE;
            spikeHeading = BlueBackstageTrajectoryBuilder.MIDDLE_SPIKE_BASE_HEADING;
        } else {
            spikePose = BACK_STAGE_OUTER_SPIKE_BASE;
            spikeHeading = BlueBackstageTrajectoryBuilder.OUTER_SPIKE_BASE_HEADING;
        }

        builder.splineToLinearHeading(spikePose, Math.toRadians(spikeHeading));

        // The second step is to park the robot

        // Set the tangent to 0 so that the robot begins it's path by driving towards the backdrop
        builder.setTangent(Math.toRadians(0));

        builder.splineToLinearHeading(BACK_STAGE_PARKING_POSITION, Math.toRadians(0));


        return builder.build();
    }
}
