package org.firstinspires.ftc.teamcode.autonomous.trajectory;

import com.acmerobotics.dashboard.config.Config;

/**
 * Common configuration shared by both the frontstage and backstage trajectories
 */
@Config
public class RedAutonomous {
    // Start pose heading
    public static double START_POSE_HEADING = 90;

    // Headings used for most of the robot's movements
    public static double SPLINE_ANGLE = 0;
    public static double SPLINE_CONSTANT_HEADING_ANGLE = 180;

    // Position to move to when approaching the backdrop
    public static double EDGE_BACKDROP_APPROACH_X = 43.5;
    public static double EDGE_BACKDROP_APPROACH_Y = -44.25;
    public static double MIDDLE_BACKDROP_APPROACH_X = EDGE_BACKDROP_APPROACH_X;
    public static double MIDDLE_BACKDROP_APPROACH_Y = -39.5;
    public static double CENTER_BACKDROP_APPROACH_X = EDGE_BACKDROP_APPROACH_X;
    public static double CENTER_BACKDROP_APPROACH_Y = -30;

    // Positions to line up for scoring
    public static double BACKDROP_APPROACH_DISTANCE = 3.0;
    public static double EDGE_BACKDROP_SCORE_X = EDGE_BACKDROP_APPROACH_X + BACKDROP_APPROACH_DISTANCE;
    public static double EDGE_BACKDROP_SCORE_Y = EDGE_BACKDROP_APPROACH_Y;
    public static double MIDDLE_BACKDROP_SCORE_X = MIDDLE_BACKDROP_APPROACH_X + BACKDROP_APPROACH_DISTANCE;
    public static double MIDDLE_BACKDROP_SCORE_Y = MIDDLE_BACKDROP_APPROACH_Y;
    public static double CENTER_BACKDROP_SCORE_X = CENTER_BACKDROP_APPROACH_X + BACKDROP_APPROACH_DISTANCE;
    public static double CENTER_BACKDROP_SCORE_Y = CENTER_BACKDROP_APPROACH_Y;

    // Positions when parking
    public static double CENTER_INTERMEDIATE_PARKING_X = 54.0;
    public static double CENTER_INTERMEDIATE_PARKING_Y = -13.0;
    public static double CENTER_PARKING_X = 58.0;
    public static double CENTER_PARKING_Y = CENTER_INTERMEDIATE_PARKING_Y;
    public static double EDGE_INTERMEDIATE_PARKING_X = 54.0;
    public static double EDGE_INTERMEDIATE_PARKING_Y = -62.5;
    public static double EDGE_PARKING_X = 58.0;
    public static double EDGE_PARKING_Y = EDGE_INTERMEDIATE_PARKING_Y;
}
