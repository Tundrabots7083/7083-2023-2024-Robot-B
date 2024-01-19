package com.example.meepmeeptesting;

import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public interface AutonomousTrajectoryBuilder {

    TrajectorySequence getTrajectory(TeamElementLocation targetLocation, DriveShim drive);

}
