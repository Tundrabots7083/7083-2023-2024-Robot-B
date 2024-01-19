package org.firstinspires.ftc.teamcode.behaviortree;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class Node {
    protected String name;
    protected Robot robot;

    public Node(String name, Robot robot) {
        this.name = name;
        this.robot = robot;
    }

    public abstract NodeStatus tick();
}