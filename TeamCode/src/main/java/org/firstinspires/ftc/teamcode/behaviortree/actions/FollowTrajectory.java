package org.firstinspires.ftc.teamcode.behaviortree.actions;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.Node;
import org.firstinspires.ftc.teamcode.behaviortree.NodeStatus;

public class FollowTrajectory extends Node {

    Trajectory trajectory;

    boolean started = false;

    public FollowTrajectory(String name, Robot robot, Trajectory trajectory) {
        super(name, robot);
        this.trajectory = trajectory;
    }

    @Override
    public NodeStatus tick() {

        if (!started) {
            robot.drive.followTrajectoryAsync(trajectory);
            started = true;
        }

        if (!robot.drive.isBusy()) {
            // TODO: Validate that we are where we want to be, and return FAILURE if not
            return NodeStatus.SUCCESS;
        }

        return NodeStatus.RUNNING;
    }
}
