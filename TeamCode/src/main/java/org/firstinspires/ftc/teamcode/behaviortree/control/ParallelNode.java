package org.firstinspires.ftc.teamcode.behaviortree.control;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.Node;
import org.firstinspires.ftc.teamcode.behaviortree.NodeStatus;

public class ParallelNode extends Node {
    private Node[] children;

    public ParallelNode(String name, Robot robot, Node[] children) {
        super(name, robot);
        this.children = children;
    }

    @Override
    public NodeStatus tick() {
        boolean allSuccess = true;
        boolean allFailure = true;
        for (Node child : children) {
            NodeStatus status = child.tick();
            if (status == NodeStatus.SUCCESS) {
                allFailure = false;
            } else if (status == NodeStatus.FAILURE) {
                allSuccess = false;
            } else {
                allSuccess = false;
                allFailure = false;
            }
        }
        if (allSuccess) {
            return NodeStatus.SUCCESS;
        } else if (allFailure) {
            return NodeStatus.FAILURE;
        } else {
            return NodeStatus.RUNNING;
        }
    }
}
