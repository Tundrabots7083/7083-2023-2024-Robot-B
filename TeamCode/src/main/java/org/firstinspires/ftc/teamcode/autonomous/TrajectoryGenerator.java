package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.processor.TeamElementLocation;

/**
 * Generates a Roadrunner trajectory to score a pixel on the spike mark, on the backdrop, and
 * then park either on the edge or the center.
 */
public interface TrajectoryGenerator {

    /**
     * Returns the starting pose for the robot.
     *
     * @return the starting pose for the robot.
     */
    Pose2d getStartingPose();

    /**
     * Move to the correct spike mark based on the location of the team prop and drop off a
     * purple pixel. This will score 20 points in autonomous.
     *
     * @param builder             the trajectory builder to use.
     * @param teamElementLocation location, on the spike mark, of the team prop.
     * @return the trajectory to move to the spike mark.
     */
    Trajectory toSpikeMark(TrajectoryBuilder builder, TeamElementLocation teamElementLocation);

    /**
     * Move to the position in front of the correct location (April Tag) on the backdrop, but
     * far enough away so that the arm can be rotated and the pixel container position in a low
     * scoring position.
     *
     * @param builder             the trajectory builder to use.
     * @param teamElementLocation location, on the spike mark, of the team prop.
     * @return the trajectory to move to the initial scoring position.
     */
    Trajectory toArmLiftPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation);

    /**
     * Move forward a default number of inches toward the backdrop to position the arm close enough
     * to score the purple pixel. This will score 20 autonomous points.
     *
     * @param builder the trajectory builder to use.
     * @return the trajectory to move to the scoring position.
     */
    Trajectory toBackdropPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation);

    /**
     * Move forward the specified number of inches toward the backdrop to position the arm close
     * enough to score the purple pixel. This method is intended to be used if a distance sensor
     * or other visual mechanism can precisely determine the distance to the backdrop. This will
     * score 20 autonomous points.
     *
     * @param builder             the trajectory builder to use.
     * @param teamElementLocation location, on the spike mark, of the team prop.
     * @return the trajectory to move to the scoring position.
     */
    Trajectory toBackdropPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation, double distance);

    /**
     * Move away from the backdrop so that the arm and pixel container can be positioned into the
     * intake position without de-scoring pixels on the backdrop.
     *
     * @param builder             the trajectory builder to use.
     * @param teamElementLocation location, on the spike mark, of the team prop.
     * @return the trajectory to move away from the backdrop.
     */
    Trajectory toArmRetractionPosition(TrajectoryBuilder builder, TeamElementLocation teamElementLocation);

    /**
     * Move to the center (middle) of the alliance backdrop and park next to it. This will score
     * 5 autonomous points
     *
     * @param builder             the trajectory builder to use.
     * @param teamElementLocation location, on the spike mark, of the team prop.
     * @return the trajectory to move to the center (middle) parking spot.
     */
    Trajectory toParkingSpotCenter(TrajectoryBuilder builder, TeamElementLocation teamElementLocation);

    /**
     * Move to the edge (glass side side) of the alliance backdrop and park next to it. This will
     * score 5 autonomous points.
     *
     * @param builder             the trajectory builder to use.
     * @param teamElementLocation location, on the spike mark, of the team prop.
     * @return the trajectory to move to the edge (glass middle) parking spot.
     */
    Trajectory toParkingSpotEdge(TrajectoryBuilder builder, TeamElementLocation teamElementLocation);
}
