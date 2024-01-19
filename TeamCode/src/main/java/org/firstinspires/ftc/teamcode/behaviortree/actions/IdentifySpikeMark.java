package org.firstinspires.ftc.teamcode.behaviortree.actions;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.Node;
import org.firstinspires.ftc.teamcode.behaviortree.NodeStatus;

public class IdentifySpikeMark extends Node {

    public IdentifySpikeMark(String name, Robot robot) {
        super(name, robot);
    }

    @Override
    public NodeStatus tick() {

        if (robot.visionSensor.webcamInitialized()) {
            robot.state.teamElementLocation = robot.visionSensor.getTeamElementLocation();
            return NodeStatus.SUCCESS;
        }

        return NodeStatus.RUNNING;
    }
}
