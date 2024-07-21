package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subsystem.DroneLauncher;

/**
 * Command to control the drone launcher. The command can raise and lower the drone launcher as
 * well as launch the drone.
 */
public class DroneLauncherCommand extends CommandBaseEx {
    private final DroneLauncher droneLauncher;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    /**
     * Instantiates a command to control the drone launcher.
     *
     * @param droneLauncher the drone launcher to control
     * @param gamepad1      Gamepad1
     * @param gamepad2      Gamepad2
     */
    public DroneLauncherCommand(DroneLauncher droneLauncher, Gamepad gamepad1, Gamepad gamepad2) {
        this.droneLauncher = droneLauncher;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        addRequirements(droneLauncher);
    }

    /**
     * Runs the drone launcher command. This looks for input from the gamepads and adjusts the
     * angle of the drone launcher and launches the drone depending on the keys pressed.
     */
    @Override
    public void execute() {
        Telemetry telemetry = MyRobot.getInstance().telemetry;

        // Set the drone launch angle based on the buttons pressed on Gamepad2
        if (gamepad2.square) {
            droneLauncher.setToLaunchAngle();
            telemetry.addLine("[Drone] Set to Launch Angle");
        } else if (gamepad2.triangle) {
            droneLauncher.setToStartAngle();
            telemetry.addLine("[Drone] Set to Initial Angle");
        }

        // Both right bumpers must be pressed at the same time to launch the drone. This is to
        // avoid an accidental drone launch.
        if (gamepad1.right_bumper && gamepad2.right_bumper) {
            droneLauncher.launchDrone();
            telemetry.addLine("[Drone] Launched");
        }
    }
}
