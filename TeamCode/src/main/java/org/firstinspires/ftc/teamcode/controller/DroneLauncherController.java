package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.controller.subsystem.DroneLauncher;

/**
 * DroneLauncherController processes the input to tell when to position the arm to the correct
 * angle and launch the drone.
 */
public class DroneLauncherController implements Controller {
    private final DroneLauncher droneLauncher;

    /**
     * Initializes the drone launcher controller.
     *
     * @param droneLauncher the drone launcher to be controlled.
     */
    public DroneLauncherController(DroneLauncher droneLauncher) {
        this.droneLauncher = droneLauncher;
    }

    /**
     * Checks to see if the right bumper is pressed and, if so, positions the arm to the correct
     * angle and then launches the drone.
     *
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        Telemetry telemetry = MyRobot.getInstance().telemetry;

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
