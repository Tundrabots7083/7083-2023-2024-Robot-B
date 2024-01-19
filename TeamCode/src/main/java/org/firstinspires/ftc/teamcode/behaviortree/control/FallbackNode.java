package org.firstinspires.ftc.teamcode.behaviortree.control;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.Node;
import org.firstinspires.ftc.teamcode.behaviortree.NodeStatus;

public class FallbackNode extends Node {
    private Node[] children;

    public FallbackNode(String name, Robot robot, Node[] children) {
        super(name, robot);
        this.children = children;
    }

    @Override
    public NodeStatus tick() {
        for (Node child : children) {
            NodeStatus status = child.tick();
            if (status != NodeStatus.FAILURE) {
                return status;
            }
        }
        return NodeStatus.FAILURE;
    }

}
