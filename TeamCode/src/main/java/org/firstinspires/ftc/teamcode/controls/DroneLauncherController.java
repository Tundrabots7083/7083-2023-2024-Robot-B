package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.DroneLauncher;

/**
 * DroneLauncherController processes the input to tell when to position the arm to the correct
 * angle and launch the drone.
 */
public class DroneLauncherController implements Controller {
    private final DroneLauncher droneLauncher = new DroneLauncher("droneLauncher", "Drone Launcher");
    private boolean droneLaunched = false;

    /**
     * Initializes the drone launcher controller.
     * @param hardwareMap the hardware map for the robot.
     */
    @Override
    public void init(HardwareMap hardwareMap) {
        droneLauncher.init(hardwareMap);
    }

    /**
     * Checks to see if the right bumper is pressed and, if so, positions the arm to the correct
     * angle and then launches the drone.
     * @param gamepad
     * @param telemetry
     */
    @Override
    public void execute(Gamepad gamepad, Telemetry telemetry) {
        boolean bumper = gamepad.right_bumper;
        if (bumper && !droneLaunched) {
            ArmController armController = Robot.getRobot().armController;
            armController.setToLauncheDrone();
            droneLauncher.launch();
            droneLaunched = true;
        }
    }
}
