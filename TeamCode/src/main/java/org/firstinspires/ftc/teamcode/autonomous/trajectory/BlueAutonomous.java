package org.firstinspires.ftc.teamcode.autonomous.trajectory;

import com.acmerobotics.dashboard.config.Config;

/**
 * Common configuration shared by both the frontstage and backstage trajectories. Most of this
 * configuration is a "mirror* of the Red Alliance settings. As such, you should avoid editing
 * the fields here and instead make changes in RedAutonomous, and only make changes here as a
 * last resort.
 */
@Config
public class BlueAutonomous {
    // Distance between the two pixel containers. Since the pixel containers are not in the middle,
    // this value is used to adjust the location of the robot so the proper container lines up on
    // the spike mark and backdrop.
    public static double PIXEL_CONTAINER_OFFSET = 5.0;

    // Starting position
    public static double START_POSE_HEADING = -RedFrontstage.START_POSE_HEADING;

    // Position to move to when approaching the backdrop
    public static double EDGE_BACKDROP_APPROACH_X = RedFrontstage.EDGE_BACKDROP_APPROACH_X;
    public static double EDGE_BACKDROP_APPROACH_Y = -RedFrontstage.EDGE_BACKDROP_APPROACH_Y - PIXEL_CONTAINER_OFFSET;
    public static double MIDDLE_BACKDROP_APPROACH_X = RedFrontstage.MIDDLE_BACKDROP_APPROACH_X;
    public static double MIDDLE_BACKDROP_APPROACH_Y = -RedFrontstage.MIDDLE_BACKDROP_SCORE_Y - PIXEL_CONTAINER_OFFSET;
    public static double CENTER_BACKDROP_APPROACH_X = RedFrontstage.CENTER_BACKDROP_APPROACH_X;
    public static double CENTER_BACKDROP_APPROACH_Y = -RedFrontstage.CENTER_BACKDROP_APPROACH_Y - PIXEL_CONTAINER_OFFSET;

    // Positions to line up for scoring
    public static double EDGE_BACKDROP_SCORE_X = RedFrontstage.EDGE_BACKDROP_SCORE_X;
    public static double EDGE_BACKDROP_SCORE_Y = EDGE_BACKDROP_APPROACH_Y;
    public static double MIDDLE_BACKDROP_SCORE_X = RedFrontstage.MIDDLE_BACKDROP_SCORE_X;
    public static double MIDDLE_BACKDROP_SCORE_Y = MIDDLE_BACKDROP_APPROACH_Y;
    public static double CENTER_BACKDROP_SCORE_X = RedFrontstage.CENTER_BACKDROP_SCORE_X;
    public static double CENTER_BACKDROP_SCORE_Y = CENTER_BACKDROP_APPROACH_Y;

    // Positions when parking
    public static double CENTER_INTERMEDIATE_PARKING_X = RedFrontstage.CENTER_INTERMEDIATE_PARKING_X;
    public static double CENTER_INTERMEDIATE_PARKING_Y = -RedFrontstage.CENTER_INTERMEDIATE_PARKING_Y;
    public static double CENTER_PARKING_X = RedFrontstage.CENTER_PARKING_X;
    public static double CENTER_PARKING_Y = -RedFrontstage.CENTER_PARKING_Y;
    public static double EDGE_INTERMEDIATE_PARKING_X = RedFrontstage.EDGE_INTERMEDIATE_PARKING_X;
    public static double EDGE_INTERMEDIATE_PARKING_Y = -RedFrontstage.EDGE_INTERMEDIATE_PARKING_Y;
    public static double EDGE_PARKING_X = RedFrontstage.EDGE_PARKING_X;
    public static double EDGE_PARKING_Y = -RedFrontstage.EDGE_PARKING_Y;
}
