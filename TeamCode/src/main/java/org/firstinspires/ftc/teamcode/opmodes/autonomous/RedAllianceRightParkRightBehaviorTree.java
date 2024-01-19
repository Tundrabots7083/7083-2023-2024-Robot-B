package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.Robot.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.behaviortree.Node;
import org.firstinspires.ftc.teamcode.behaviortree.actions.IdentifySpikeMark;

import java.util.ArrayList;

public class RedAllianceRightParkRightBehaviorTree extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Create the behavior tree nodes
        ArrayList<Node> nodes = new ArrayList<>();
        nodes.add(new IdentifySpikeMark("Identify Spike Mark", robot));

        // Initialize the robot
        Robot robot = new Robot(hardwareMap, telemetry);

        // Initialize the webcam
        robot.visionSensor.initializeVisionPortal();

        while(!robot.visionSensor.webcamInitialized()) {
            // Wait for webcam to initialize
            telemetry.addData("Webcam", "Initializing...");
            telemetry.update();
        }


        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
