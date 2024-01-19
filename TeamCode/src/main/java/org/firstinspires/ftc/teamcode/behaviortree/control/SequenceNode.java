package org.firstinspires.ftc.teamcode.behaviortree.control;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.Node;
import org.firstinspires.ftc.teamcode.behaviortree.NodeStatus;

public class SequenceNode extends Node {
    private Node[] children;

    public SequenceNode(String name, Robot robot, Node[] children) {
        super(name, robot);
        this.children = children;
    }

    @Override
    public NodeStatus tick() {
        for (Node child : children) {
            NodeStatus status = child.tick();
            if (status != NodeStatus.SUCCESS) {
                return status;
            }
        }
        return NodeStatus.SUCCESS;
    }
}
