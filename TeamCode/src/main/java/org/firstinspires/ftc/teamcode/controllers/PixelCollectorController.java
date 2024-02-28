package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;

/**
 * Controller for the pixel collectors on the robot.
 */
public class PixelCollectorController implements Controller {

    private final Telemetry telemetry;
    private final PixelCollector leftPixelCollector;
    private final PixelCollector rightPixelCollector;
    private final Gamepad previousGamepad2 = new Gamepad();

    /**
     * Creates a new pixel collector controller.
     * @param leftPixelCollector the left pixel collector.
     * @param rightPixelCollector the right pixel collector.
     * @param telemetry the telemetry used to display output on the driver station.
     */
    public PixelCollectorController(PixelCollector leftPixelCollector, PixelCollector rightPixelCollector, Telemetry telemetry) {
        this.rightPixelCollector = leftPixelCollector;
        this.leftPixelCollector = rightPixelCollector;

        this.telemetry = telemetry;
    }

    /**
     * Process input to the pixel collector controller each loop through the opmode.
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Gamepad 2 will control both pixel collectors
        // dpad down and dpad right will control the left pixel collector's state
        // a and b will be used for the right pixel collector's state

        // Left pixel collector
        if (!previousGamepad2.dpad_down && gamepad2.dpad_down) {
            telemetry.addLine("[LEFT PC] Set to collecting");
            // Toggle collection on or off
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.COLLECTING);
        } else if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
            telemetry.addLine("[LEFT PC] Set to depositing");
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
            // Toggle depositing on or off
        } else if (!gamepad2.dpad_down && !gamepad2.dpad_right && (previousGamepad2.dpad_down || previousGamepad2.dpad_right)) {
            telemetry.addLine("[LEFT PC] Set to closed");
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        }

        // Right pixel collector
        if (!previousGamepad2.a && gamepad2.a) {
            telemetry.addLine("[RIGHT PC] Set to collecting");
            // Toggle collection on or off
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.COLLECTING);
        } else if (!previousGamepad2.b && gamepad2.b) {
            telemetry.addLine("[LEFT PC] Set to depositing");
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
            // Toggle depositing on or off
        } else if (!gamepad2.a && !gamepad2.b && (previousGamepad2.a || previousGamepad2.b)) {
            telemetry.addLine("[LEFT PC] Set to closed");
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        }

        leftPixelCollector.update();
        rightPixelCollector.update();

        previousGamepad2.copy(gamepad2);
    }
}
