package org.firstinspires.ftc.teamcode.autonomous.trajectories;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.field.TeamElementLocation;

@Config
public class RedBackstageTrajectoryGenerator implements TrajectoryGenerator {
    public static final Pose2d STARTING_POSE = new Pose2d(15.25, -63.125, Math.toRadians(90));

    public static final Pose2d INNER_SPIKE_BASE = new Pose2d(13.25, -34.5, Math.toRadians(180));
    public static final int INNER_SPIKE_BASE_HEADING = 180;
    public static final Pose2d MIDDLE_SPIKE_BASE = new Pose2d(13.75, -37.5, Math.toRadians(90));
    public static final int MIDDLE_SPIKE_BASE_HEADING = 90;
    public static final Pose2d OUTER_SPIKE_BASE = new Pose2d(27, -47, Math.toRadians(90));
    public static final int OUTER_SPIKE_BASE_HEADING = 90;

    public static final Vector2d BACKDROP_INTERMEDIATE_POSITION = new Vector2d(40.5, -43);
    public static final Vector2d BACKDROP_EDGE_POSITION = new Vector2d(49, -44.25);
    public static final Vector2d BACKDROP_MIDDLE_POSITION = new Vector2d(49, -39.5);
    public static final Vector2d BACKDROP_CENTER_POSITION = new Vector2d(47, -30);
    public static final Vector2d REVERSE_POSITION = new Vector2d(27, -59);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION_CENTER = new Vector2d(45, -16);
    public static final Vector2d PARKING_POSITION_CENTER = new Vector2d(59, -11);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION_EDGE = new Vector2d(45, -57);
    public static final Vector2d PARKING_POSITION_EDGE = new Vector2d(59, -59);
    public static double BACKDROP_CENTER_ROTATE = 1;
    public static double BACKDROP_MIDDLE_ROTATE = 95;
    public static double BACKDROP_EDGE_ROTATE = 91.5;
    public static double BACKDROP_FORWARD_DISTANCE = 3;
    public static double BACKDROP_BACKWARD_DISTANCE = 3.5;

    @Override
    public Pose2d getStartingPose() {
        return STARTING_POSE;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        // The first step is to drive the robot from the starting position to the correct spike mark.
        switch (teamElementLocation) {
            case LEFT_SPIKE_MARK:
                builder.splineToLinearHeading(INNER_SPIKE_BASE, Math.toRadians(INNER_SPIKE_BASE_HEADING));
                break;
            case MIDDLE_SPIKE_MARK:
                builder.splineToLinearHeading(MIDDLE_SPIKE_BASE, Math.toRadians(MIDDLE_SPIKE_BASE_HEADING));
                break;
            default:
                builder.splineToLinearHeading(OUTER_SPIKE_BASE, Math.toRadians(OUTER_SPIKE_BASE_HEADING));
        }
        return builder.build();
    }

    @Override
    public Trajectory toArmLiftPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation) {
        switch (teamElementLocation) {
            case LEFT_SPIKE_MARK:
                builder.splineToConstantHeading(BACKDROP_CENTER_POSITION, Math.toRadians(BACKDROP_CENTER_ROTATE));
                break;
            case MIDDLE_SPIKE_MARK:
                builder.splineToConstantHeading(REVERSE_POSITION, Math.toRadians(0))
                        .splineTo(BACKDROP_INTERMEDIATE_POSITION, Math.toRadians(BACKDROP_MIDDLE_ROTATE))
                        .splineToConstantHeading(BACKDROP_MIDDLE_POSITION, Math.toRadians(0));
                break;
            default:
                builder.splineToConstantHeading(REVERSE_POSITION, Math.toRadians(0))
                        .splineTo(BACKDROP_INTERMEDIATE_POSITION, Math.toRadians(BACKDROP_EDGE_ROTATE))
                        .splineToConstantHeading(BACKDROP_EDGE_POSITION, Math.toRadians(0));

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

    @Override
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
