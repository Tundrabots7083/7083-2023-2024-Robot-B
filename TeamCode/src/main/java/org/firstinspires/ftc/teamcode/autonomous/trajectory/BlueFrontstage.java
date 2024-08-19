package org.firstinspires.ftc.teamcode.autonomous.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.ScoringSubsystem;

/**
 * Blue Alliance frontstage trajectories. These are a "mirror" representation of the Red Alliance
 * trajectories. As such, changes should be made to the red alliance trajectories.
 */
@Config
public class BlueFrontstage extends BlueAutonomous {
    public static double DELAY_OPMODE = RedFrontstage.DELAY_OPMODE; // In secondd

    // Starting pose
    public static double START_POSE_X = RedFrontstage.START_POSE_X;
    public static double START_POSE_Y = -RedBackstage.START_POSE_Y;

    // Starting pose for the robot
    public static final Pose2d STARTING_POSE = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_HEADING));

    // Spike mark locations
    public static double OUTER_SPIKE_MARK_X = RedFrontstage.OUTER_SPIKE_MARK_X - PIXEL_CONTAINER_OFFSET;
    public static double OUTER_SPIKE_MARK_Y = -RedFrontstage.OUTER_SPIKE_MARK_Y;
    public static double MIDDLE_SPIKE_MARK_X = RedFrontstage.MIDDLE_SPIKE_MARK_X - PIXEL_CONTAINER_OFFSET;
    public static double MIDDLE_SPIKE_MARK_Y = -RedFrontstage.MIDDLE_SPIKE_MARK_Y;
    public static double INNER_SPIKE_MARK_X = RedFrontstage.MIDDLE_SPIKE_MARK_X - PIXEL_CONTAINER_OFFSET;
    public static double INNER_SPIKE_MARK_Y = -RedFrontstage.MIDDLE_SPIKE_MARK_Y;
    public static double INNER_SPIKE_MARK_HEADING = -RedFrontstage.INNER_SPIKE_MARK_HEADING;

    // Headings used for most of the robot's movements
    public static double SPLINE_ANGLE = -RedFrontstage.SPLINE_ANGLE;
    public static double SPLINE_CONSTANT_HEADING_ANGLE = -RedFrontstage.SPLINE_CONSTANT_HEADING_ANGLE;

    // Position on frontstage side before moving under rigging
    public static double FRONTSTAGE_UNDER_RIG_X = RedFrontstage.FRONTSTAGE_UNDER_RIG_X;
    public static double FRONTSTAGE_UNDER_RIG_Y = -RedFrontstage.FRONTSTAGE_UNDER_RIG_Y;

    // Position on the backstage side after moving under the rigging
    public static double BACKSTAGE_UNDER_RIG_X = RedFrontstage.BACKSTAGE_UNDER_RIG_X;
    public static double BACKSTAGE_UNDER_RIG_Y = -RedFrontstage.BACKSTAGE_UNDER_RIG_Y;

    public final Action outerSpikeMarkParkCenter;
    public final Action outerSpikeMarkParkBackdrop;
    public final Action outerSpikeMarkParkEdge;
    public final Action middleSpikeMarkParkCenter;
    public final Action middleSpikeMarkParkBackdrop;
    public final Action middleSpikeMarkParkEdge;
    public final Action innerSpikeMarkParkCenter;
    public final Action innerSpikeMarkParkBackdrop;
    public final Action innerSpikeMarkParkEdge;

    public BlueFrontstage(AutoMecanumDrive mecanumDrive) {
        // Right Spike Mark trajectories
        Action outerMoveToSpikeMark = mecanumDrive.actionBuilder(STARTING_POSE)
                .waitSeconds(DELAY_OPMODE)
                .strafeTo(new Vector2d(OUTER_SPIKE_MARK_X, OUTER_SPIKE_MARK_Y))
                .build();
        Action outerMoveToBackdropApproach = mecanumDrive.actionBuilder(new Pose2d(OUTER_SPIKE_MARK_X, OUTER_SPIKE_MARK_Y, Math.toRadians(START_POSE_HEADING)))
                .setReversed(true)
                .splineTo(new Vector2d(FRONTSTAGE_UNDER_RIG_X, FRONTSTAGE_UNDER_RIG_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(BACKSTAGE_UNDER_RIG_X, BACKSTAGE_UNDER_RIG_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(EDGE_BACKDROP_APPROACH_X, EDGE_BACKDROP_APPROACH_Y), Math.toRadians(SPLINE_ANGLE))
                .build();
        Action outerMoveToScore = mecanumDrive.actionBuilder(new Pose2d(EDGE_BACKDROP_APPROACH_X, EDGE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .strafeTo(new Vector2d(EDGE_BACKDROP_SCORE_X, EDGE_BACKDROP_SCORE_Y))
                .build();
        Action outerMoveToLowerLift = mecanumDrive.actionBuilder(new Pose2d(EDGE_BACKDROP_SCORE_X, EDGE_BACKDROP_SCORE_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .strafeTo(new Vector2d(EDGE_BACKDROP_APPROACH_X, EDGE_BACKDROP_APPROACH_Y))
                .build();
        Action outerParkCenter = mecanumDrive.actionBuilder(new Pose2d(EDGE_BACKDROP_APPROACH_X, EDGE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .splineToConstantHeading(new Vector2d(CENTER_INTERMEDIATE_PARKING_X, CENTER_INTERMEDIATE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(CENTER_PARKING_X, CENTER_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .build();
        Action outerParkBackdrop = mecanumDrive.actionBuilder(new Pose2d(EDGE_BACKDROP_APPROACH_X, EDGE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .build();
        Action outerParkEdge = mecanumDrive.actionBuilder(new Pose2d(EDGE_BACKDROP_APPROACH_X, EDGE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .splineToConstantHeading(new Vector2d(EDGE_INTERMEDIATE_PARKING_X, EDGE_INTERMEDIATE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(EDGE_PARKING_X, EDGE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .build();

        // Middle Spike Mark trajectories
        Action middleMoveToSpikeMark = mecanumDrive.actionBuilder(STARTING_POSE)
                .waitSeconds(DELAY_OPMODE)
                .strafeTo(new Vector2d(MIDDLE_SPIKE_MARK_X, MIDDLE_SPIKE_MARK_Y))
                .build();
        Action middleMoveToBackdropApproach = mecanumDrive.actionBuilder(new Pose2d(MIDDLE_SPIKE_MARK_X, MIDDLE_SPIKE_MARK_Y, Math.toRadians(START_POSE_HEADING)))
                .setReversed(true)
                .splineTo(new Vector2d(FRONTSTAGE_UNDER_RIG_X, FRONTSTAGE_UNDER_RIG_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(BACKSTAGE_UNDER_RIG_X, BACKSTAGE_UNDER_RIG_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(MIDDLE_BACKDROP_APPROACH_X, MIDDLE_BACKDROP_APPROACH_Y), Math.toRadians(SPLINE_ANGLE))
                .build();
        Action middleMoveToScore = mecanumDrive.actionBuilder(new Pose2d(MIDDLE_BACKDROP_APPROACH_X, MIDDLE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .strafeTo(new Vector2d(MIDDLE_BACKDROP_SCORE_X, MIDDLE_BACKDROP_SCORE_Y))
                .build();
        Action middleMoveToLowerLift = mecanumDrive.actionBuilder(new Pose2d(MIDDLE_BACKDROP_SCORE_X, MIDDLE_BACKDROP_SCORE_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .strafeTo(new Vector2d(MIDDLE_BACKDROP_APPROACH_X, MIDDLE_BACKDROP_APPROACH_Y))
                .build();
        Action middleParkCenter = mecanumDrive.actionBuilder(new Pose2d(MIDDLE_BACKDROP_APPROACH_X, MIDDLE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .splineToConstantHeading(new Vector2d(CENTER_INTERMEDIATE_PARKING_X, CENTER_INTERMEDIATE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(CENTER_PARKING_X, CENTER_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .build();
        Action middleParkBackdrop = mecanumDrive.actionBuilder(new Pose2d(MIDDLE_BACKDROP_APPROACH_X, MIDDLE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .build();
        Action middleParkEdge = mecanumDrive.actionBuilder(new Pose2d(MIDDLE_BACKDROP_APPROACH_X, MIDDLE_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .splineToConstantHeading(new Vector2d(EDGE_INTERMEDIATE_PARKING_X, EDGE_INTERMEDIATE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(EDGE_PARKING_X, EDGE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .build();

        // Left Spike Mark trajectories
        Action innerMoveToSpikeMark = mecanumDrive.actionBuilder(STARTING_POSE)
                .waitSeconds(DELAY_OPMODE)
                .splineTo(new Vector2d(INNER_SPIKE_MARK_X, INNER_SPIKE_MARK_Y), Math.toRadians(INNER_SPIKE_MARK_HEADING))
                .build();
        Action innerMoveToBackdropApproach = mecanumDrive.actionBuilder(new Pose2d(INNER_SPIKE_MARK_X, INNER_SPIKE_MARK_Y, Math.toRadians(INNER_SPIKE_MARK_HEADING)))
                .setReversed(true)
                .splineTo(new Vector2d(FRONTSTAGE_UNDER_RIG_X, FRONTSTAGE_UNDER_RIG_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(BACKSTAGE_UNDER_RIG_X, BACKSTAGE_UNDER_RIG_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(CENTER_BACKDROP_APPROACH_X, CENTER_BACKDROP_APPROACH_Y), Math.toRadians(SPLINE_ANGLE))
                .build();
        Action innerMoveToScore = mecanumDrive.actionBuilder(new Pose2d(CENTER_BACKDROP_APPROACH_X, CENTER_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .strafeTo(new Vector2d(CENTER_BACKDROP_SCORE_X, CENTER_BACKDROP_SCORE_Y))
                .build();
        Action innerMoveToLowerLift = mecanumDrive.actionBuilder(new Pose2d(CENTER_BACKDROP_SCORE_X, CENTER_BACKDROP_SCORE_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .strafeTo(new Vector2d(CENTER_BACKDROP_APPROACH_X, CENTER_BACKDROP_APPROACH_Y))
                .build();
        Action innerParkCenter = mecanumDrive.actionBuilder(new Pose2d(CENTER_BACKDROP_APPROACH_X, CENTER_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .splineToConstantHeading(new Vector2d(CENTER_INTERMEDIATE_PARKING_X, CENTER_INTERMEDIATE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(CENTER_PARKING_X, CENTER_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .build();
        Action innerParkBackdrop = mecanumDrive.actionBuilder(new Pose2d(CENTER_BACKDROP_APPROACH_X, CENTER_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .build();
        Action innerParkEdge = mecanumDrive.actionBuilder(new Pose2d(CENTER_BACKDROP_APPROACH_X, CENTER_BACKDROP_APPROACH_Y, Math.toRadians(SPLINE_CONSTANT_HEADING_ANGLE)))
                .splineToConstantHeading(new Vector2d(EDGE_INTERMEDIATE_PARKING_X, EDGE_INTERMEDIATE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .splineToConstantHeading(new Vector2d(EDGE_PARKING_X, EDGE_PARKING_Y), Math.toRadians(SPLINE_ANGLE))
                .build();

        Robot robot = Robot.getInstance();
        // Full trajectories for Frontstage
        innerSpikeMarkParkCenter = new SequentialAction(
                innerMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                innerMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                innerMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                innerMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                innerParkCenter
        );
        innerSpikeMarkParkBackdrop = new SequentialAction(
                innerMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                innerMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                innerMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                innerMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                innerParkBackdrop
        );
        innerSpikeMarkParkEdge = new SequentialAction(
                innerMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                innerMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                innerMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                innerMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                innerParkEdge
        );

        outerSpikeMarkParkCenter = new SequentialAction(
                outerMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                outerMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                outerMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                outerMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                outerParkCenter
        );
        outerSpikeMarkParkBackdrop = new SequentialAction(
                outerMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                outerMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                outerMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                outerMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                outerParkBackdrop
        );
        outerSpikeMarkParkEdge = new SequentialAction(
                outerMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                outerMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                outerMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                outerMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                outerParkEdge
        );

        middleSpikeMarkParkCenter = new SequentialAction(
                middleMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                middleMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                middleMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                middleMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                middleParkCenter
        );
        middleSpikeMarkParkBackdrop = new SequentialAction(
                middleMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                middleMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                middleMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                middleMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                middleParkBackdrop
        );
        middleSpikeMarkParkEdge = new SequentialAction(
                middleMoveToSpikeMark,
                robot.leftPixelCollector.depositPixel(),
                middleMoveToBackdropApproach,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_LOW),
                middleMoveToScore,
                robot.rightPixelCollector.depositPixel(),
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.SCORE_MEDIUM),
                middleMoveToLowerLift,
                robot.scoringSubsystem.setTo(ScoringSubsystem.Position.INTAKE),
                middleParkEdge
        );
    }
}
