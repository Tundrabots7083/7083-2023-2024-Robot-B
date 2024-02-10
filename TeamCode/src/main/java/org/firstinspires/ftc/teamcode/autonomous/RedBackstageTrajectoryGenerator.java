package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;

@Config
public class RedBackstageTrajectoryGenerator implements TrajectoryGenerator {

    public static final Pose2d INNER_SPIKE_BASE = new Pose2d(15, -35, Math.toRadians(180));
    public static final int INNER_SPIKE_BASE_HEADING = 180;
    public static final Pose2d MIDDLE_SPIKE_BASE = new Pose2d(15.25, -37.5, Math.toRadians(90));
    public static final int MIDDLE_SPIKE_BASE_HEADING = 90;
    public static final Pose2d OUTER_SPIKE_BASE = new Pose2d(27, -46, Math.toRadians(90));
    public static final int OUTER_SPIKE_BASE_HEADING = 90;

    public static final Vector2d BACKDROP_INTERMEDIATE_POSITION = new Vector2d(40.5, -43);
    public static final Vector2d BACKDROP_EDGE_POSITION = new Vector2d(48, -43);
    public static final Vector2d BACKDROP_MIDDLE_POSITION = new Vector2d(46.5, -39);
    public static final Vector2d BACKDROP_CENTER_POSITION = new Vector2d(46, -29.5);
    public static double BACKDROP_MIDDLE_ROTATE = 93;
    public static double BACKDROP_CENTER_ROTATE = 0;
    public static double BACKDROP_EDGE_ROTATE = 94;

    public static double BACKDROP_FORWARD_DISTANCE = 3;
    public static double BACKDROP_BACKWARD_DISTANCE = 3.5;

    public static final Vector2d REVERSE_POSITION = new Vector2d(27, -59);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION_CENTER = new Vector2d(45, -16);
    public static final Vector2d PARKING_POSITION_CENTER = new Vector2d(59, -11);
    public static final Vector2d INTERMEDIATE_PARKING_POSITION_EDGE = new Vector2d(45, -57);
    public static final Vector2d PARKING_POSITION_EDGE = new Vector2d(59, -59);

    private final TeamElementLocation targetLocation;

    public RedBackstageTrajectoryGenerator(TeamElementLocation targetLocation) {
        this.targetLocation = targetLocation;
    }

    @Override
    public Trajectory toSpikeMark(TrajectoryBuilder builder) {
        // The first step is to drive the robot from the starting position to the correct spike mark.
        if (targetLocation == TeamElementLocation.LEFT) {
            return builder.splineToLinearHeading(INNER_SPIKE_BASE, Math.toRadians(INNER_SPIKE_BASE_HEADING))
                    .build();
        } else if (targetLocation == TeamElementLocation.MIDDLE) {
            return builder.splineToLinearHeading(MIDDLE_SPIKE_BASE, Math.toRadians(MIDDLE_SPIKE_BASE_HEADING))
                    .build();
        } else {
            return builder.splineToLinearHeading(OUTER_SPIKE_BASE, Math.toRadians(OUTER_SPIKE_BASE_HEADING))
                    .build();
        }
    }

    @Override
    public Trajectory toArmLiftPosition(TrajectoryBuilder builder) {
        switch (targetLocation) {
            case LEFT:
                builder.splineToConstantHeading(BACKDROP_CENTER_POSITION, Math.toRadians(BACKDROP_CENTER_ROTATE));
                break;
            case MIDDLE:
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
    public Trajectory toBackdropPosition(TrajectoryBuilder builder) {
        return toBackdropPosition(builder, BACKDROP_FORWARD_DISTANCE);
    }

    @Override
    public Trajectory toBackdropPosition(TrajectoryBuilder builder, double distance) {
        return builder.back(distance)
                .build();
    }

    @Override
    public Trajectory toArmRetractionPosition(TrajectoryBuilder builder) {
        return builder.forward(BACKDROP_BACKWARD_DISTANCE)
                .build();
    }

    @Override
    public Trajectory toParkingSpotCenter(TrajectoryBuilder builder) {
        return builder.splineToConstantHeading(INTERMEDIATE_PARKING_POSITION_CENTER, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_CENTER, Math.toRadians(0))
                .build();
    }

    @Override
    public Trajectory toParkingSpotEdge(TrajectoryBuilder builder) {
        return builder.splineToConstantHeading(INTERMEDIATE_PARKING_POSITION_EDGE, Math.toRadians(0))
                .splineToConstantHeading(PARKING_POSITION_EDGE, Math.toRadians(0))
                .build();
    }
}
