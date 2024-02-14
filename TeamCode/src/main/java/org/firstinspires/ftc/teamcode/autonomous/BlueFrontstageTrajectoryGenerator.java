package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

@Config
public class BlueFrontstageTrajectoryGenerator implements TrajectoryGenerator {
    public static final Pose2d STARTING_POSE = new Pose2d(-36, 63.5, Math.toRadians(-90));

    public static final Vector2d INNER_SPIKE_MARK_INTERMEDIATE_POSITION = new Vector2d(-40, 35);
    public static final Vector2d INNER_SPIKE_POSITION = new Vector2d(-35, 32);
    public static final Pose2d MIDDLE_SPIKE_POSITION = new Pose2d(-36, 37.5, Math.toRadians(-90));
    public static final int MIDDLE_SPIKE_BASE_HEADING = -90;
    public static final Pose2d OUTER_SPIKE_POSITION = new Pose2d(-47.5, 47, Math.toRadians(-90));
    public static final int OUTER_SPIKE_BASE_HEADING = -90;

    public static final Vector2d REVERSE_POSITION = new Vector2d(-36, 60);

    public static final Vector2d UNDER_STAGE_TARGET_POSITION = new Vector2d(-12, 61);
    public static final Vector2d BACKDROP_INTERMEDIATE_POSITION = new Vector2d(34, 60);

    public static final Vector2d BACKDROP_EDGE_POSITION = new Vector2d(45, 42.5);
    public static final Vector2d BACKDROP_MIDDLE_POSITION = new Vector2d(45, 35.5);
    public static final Vector2d BACKDROP_CENTER_POSITION = new Vector2d(45, 31.5);
    public static double BACKDROP_MIDDLE_ROTATE = 0;
    public static double BACKDROP_CENTER_ROTATE = 0;
    public static double BACKDROP_EDGE_ROTATE = -1;

    public static double BACKDROP_FORWARD_DISTANCE = 3;
    public static double BACKDROP_BACKWARD_DISTANCE = 3.0;

    public static final Vector2d INTERMEDIATE_PARKING_POSITION_CENTER = new Vector2d(45, 16);
    public static final Vector2d PARKING_POSITION_CENTER = new Vector2d(59, 13);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION_EDGE = new Vector2d(45, 60);
    public static final Vector2d PARKING_POSITION_EDGE = new Vector2d(59, 61);

    @Override
    public Pose2d getStartingPose() {
        return STARTING_POSE;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        // The first step is to drive the robot from the starting position to the correct spike mark.
        switch (teamElementLocation) {
            case LEFT:
                builder.splineTo(INNER_SPIKE_MARK_INTERMEDIATE_POSITION, Math.toRadians(0))
                        .splineToConstantHeading(INNER_SPIKE_POSITION, Math.toRadians(0));
                break;
            case MIDDLE:
                builder.splineToLinearHeading(MIDDLE_SPIKE_POSITION, Math.toRadians(MIDDLE_SPIKE_BASE_HEADING));
                break;
            default:
                builder.splineToLinearHeading(OUTER_SPIKE_POSITION, Math.toRadians(OUTER_SPIKE_BASE_HEADING));
        }
        return builder.build();
    }

    @Override
    public Trajectory toArmLiftPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        builder.splineTo(REVERSE_POSITION, Math.toRadians(0))
                .splineToConstantHeading(UNDER_STAGE_TARGET_POSITION, Math.toRadians(0))
                .splineToConstantHeading(BACKDROP_INTERMEDIATE_POSITION, Math.toRadians(0));

        switch (teamElementLocation) {
            case LEFT:
                builder.splineToConstantHeading(BACKDROP_EDGE_POSITION, Math.toRadians(BACKDROP_EDGE_ROTATE));
                break;
            case MIDDLE:
                builder.splineToConstantHeading(BACKDROP_MIDDLE_POSITION, Math.toRadians(BACKDROP_MIDDLE_ROTATE));
                break;
            default:
                builder.splineToConstantHeading(BACKDROP_CENTER_POSITION, Math.toRadians(BACKDROP_CENTER_ROTATE));
        }

        return builder.build();
    }

    @Override
    public Trajectory toBackdropPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        return toBackdropPosition(builder, teamElementLocation, BACKDROP_FORWARD_DISTANCE);
    }

    @Override
    public Trajectory toBackdropPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation, double distance) {
        return builder.back(distance)
                .build();
    }

    @Override
    public Trajectory toArmRetractionPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        return builder.forward(BACKDROP_BACKWARD_DISTANCE)
                .build();
    }

    public Trajectory toParkingSpotCenter(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        return builder.splineToConstantHeading(INTERMEDIATE_PARKING_POSITION_CENTER, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_CENTER, Math.toRadians(0))
                .build();
    }

    @Override
    public Trajectory toParkingSpotEdge(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        return builder.splineToConstantHeading(INTERMEDIATE_PARKING_POSITION_EDGE, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_EDGE, Math.toRadians(0))
                .build();
    }
}