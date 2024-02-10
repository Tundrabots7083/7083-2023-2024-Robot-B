package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.DroneLauncher;

/**
 * DroneLauncherController processes the input to tell when to position the arm to the correct
 * angle and launch the drone.
 */
public class DroneLauncherController implements Controller {
    private final DroneLauncher droneLauncher;
    private final Telemetry telemetry;

    /**
     * Initializes the drone launcher controller.
     * @param hardwareMap the hardware map for the robot.
     */
    public DroneLauncherController(HardwareMap hardwareMap, Telemetry telemetry) {
        droneLauncher = new DroneLauncher("droneLauncher", "Drone Launcher", hardwareMap);
        this.telemetry = telemetry;
    }

    /**
     * Checks to see if the right bumper is pressed and, if so, positions the arm to the correct
     * angle and then launches the drone.
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.x) {
            droneLauncher.setToLaunchAngle();
            telemetry.addLine("[Drone] Set to Launch Angle");
        } else if (gamepad2.y) {
            droneLauncher.setToStartAngle();
            telemetry.addLine("[Drone] Set to Initial Angle");
        }
        if (gamepad1.right_bumper && gamepad2.right_bumper) {
            droneLauncher.launchDrone();
            telemetry.addLine("[Drone] Launched");
        }
    }
}
