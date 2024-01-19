package org.firstinspires.ftc.teamcode.behaviortree;

public class BehaviorTree {
    private Node root;

    public BehaviorTree(Node root) {
        this.root = root;
    }

    public void tick() {
        root.tick();
    }
}
