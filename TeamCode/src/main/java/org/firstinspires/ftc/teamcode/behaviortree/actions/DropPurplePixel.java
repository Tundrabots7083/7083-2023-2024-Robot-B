package org.firstinspires.ftc.teamcode.behaviortree.actions;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.Node;
import org.firstinspires.ftc.teamcode.behaviortree.NodeStatus;

public class DropPurplePixel extends Node {


        static final int DROP_TIME_MS = 2000;

        private long dropStartTime;

        private boolean started = false;

        public DropPurplePixel(String name, Robot robot) {
            super(name, robot);
        }

        @Override
        public NodeStatus tick() {

            long currentTime = System.currentTimeMillis();

            if (!started) {
                dropStartTime = currentTime;
                started = true;
                robot.pixelMoverController.dropOffPixels();
            }

            if (currentTime - dropStartTime < DROP_TIME_MS) {
                return NodeStatus.RUNNING;
            }
            return NodeStatus.SUCCESS;
        }

}
