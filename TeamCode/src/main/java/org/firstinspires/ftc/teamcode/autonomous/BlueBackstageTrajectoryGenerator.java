package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

public class BlueBackstageTrajectoryGenerator implements TrajectoryGenerator {


    public static final Pose2d BACK_STAGE_INNER_SPIKE_BASE = new Pose2d(11, 32, Math.toRadians(180));
    public static final int INNER_SPIKE_BASE_HEADING = 180;
    public static final Pose2d BACK_STAGE_MIDDLE_SPIKE_BASE = new Pose2d(12, 34, Math.toRadians(-90));
    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;
    public static final Pose2d BACK_STAGE_OUTER_SPIKE_BASE = new Pose2d(23, 40, Math.toRadians(-90));
    public static final int OUTER_SPIKE_BASE_HEADING = -90;
    public static final Pose2d BACK_STAGE_PARKING_POSITION = new Pose2d(59, 59, Math.toRadians(180));

    TeamElementLocation targetLocation;

    public BlueBackstageTrajectoryGenerator(TeamElementLocation targetLocation) {
        this.targetLocation = targetLocation;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder) {

        // The first step is to drive the robot from the starting position to the correct spike mark.
        Pose2d spikePose;
        int spikeHeading = 0;
        if (targetLocation == TeamElementLocation.INNER) {
            spikePose = BACK_STAGE_INNER_SPIKE_BASE;
            spikeHeading = INNER_SPIKE_BASE_HEADING;
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            spikePose = BACK_STAGE_MIDDLE_SPIKE_BASE;
            spikeHeading = MIDDLE_SPIKE_BASE_HEADING;
        } else {
            spikePose = BACK_STAGE_OUTER_SPIKE_BASE;
            spikeHeading = OUTER_SPIKE_BASE_HEADING;
        }
        builder.splineToLinearHeading(spikePose, Math.toRadians(spikeHeading));
        return builder.build();
    }

    @Override
    public Trajectory toParkingSpot(TrajectoryBuilder builder) {
        builder.splineToLinearHeading(BACK_STAGE_PARKING_POSITION, Math.toRadians(0));
        return builder.build();
    }


}
