package org.firstinspires.ftc.teamcode.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;

/**
 * Controller for a pixel collector. This class takes input from the gamepads and translates
 * them to behavior for the managed pixel controller.
 */
@Config
public class PixelCollectorController implements Controller {
    public static long FLAP_DELAY = 250;
    public static long SPINNER_DELAY = 250;

    private final PixelCollector pixelCollector;
    private long flapTime;
    private long spinnerTime;

    /**
     * Creates a new controller to manage the left and right pixel collectors.
     *
     * @param pixelCollector the pixel collector to control.
     */
    public PixelCollectorController(PixelCollector pixelCollector) {
        this.pixelCollector = pixelCollector;
    }

    /**
     * Updates the flap position and spinner speed based on the buttons pressed on the gamepad.
     *
     * @param intakePressed     the intake button is pressed
     * @param depositingPressed the depositing button is pressed
     */
    private void setState(boolean intakePressed, boolean depositingPressed) {
        Telemetry telemetry = Robot.getInstance().telemetry;
        String collectorName = "[PC " + pixelCollector.getLocation() + "]";

        if (intakePressed) {
            // If the flap is not opened, and then star the spinner. If opening the flap, wait
            // before starting the spinner to ensure there is enough time for the flap to open
            // before starting the spinner.
            if (pixelCollector.getFlapPosition() != PixelCollector.FLAP_OPENED_POSITION) {
                pixelCollector.setFlapPosition(PixelCollector.FLAP_OPENED_POSITION);
                flapTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - flapTime > FLAP_DELAY) {
                pixelCollector.setSpinnerPower(PixelCollector.SPINNER_COLLECTING_POWER);
            }
        } else if (depositingPressed) {
            // If the flap is not opened, and then star the spinner. If opening the flap, wait
            // before starting the spinner to ensure there is enough time for the flap to open
            // before starting the spinner.
            if (pixelCollector.getFlapPosition() != PixelCollector.FLAP_OPENED_POSITION) {
                pixelCollector.setFlapPosition(PixelCollector.FLAP_OPENED_POSITION);
                flapTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - flapTime > FLAP_DELAY) {
                pixelCollector.setSpinnerPower(PixelCollector.SPINNER_DEPOSITING_POWER);
            }
        } else {
            // Turn off the spinner and close the flap. Ensure the spinner is stopped before
            // closing the flap.
            if (pixelCollector.getSpinnerPower() != PixelCollector.SPINNER_OFF_POWER) {
                pixelCollector.setSpinnerPower(PixelCollector.SPINNER_OFF_POWER);
                spinnerTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - spinnerTime > SPINNER_DELAY &&
                    pixelCollector.getFlapPosition() != PixelCollector.FLAP_CLOSED_POSITION) {
                pixelCollector.setFlapPosition(PixelCollector.FLAP_CLOSED_POSITION);
                flapTime = System.currentTimeMillis();
            }
        }
    }

    /**
     * Update the state of the pixel collector.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        switch (pixelCollector.getLocation()) {
            case LEFT:
                setState(gamepad2.dpad_down, gamepad2.dpad_right);
                break;
            case RIGHT:
                setState(gamepad2.a, gamepad2.b);
                break;
        }
    }
}
