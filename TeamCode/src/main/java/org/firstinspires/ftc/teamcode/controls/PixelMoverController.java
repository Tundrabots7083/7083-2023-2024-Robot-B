package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;

public class PixelMoverController implements Controller {
    private boolean aButtonPressed = false;
    private boolean bButtonPressed = false;

    private final PixelMover pixelMover;
    private final Telemetry telemetry;

    public PixelMoverController(HardwareMap hardwareMap, Telemetry telemetry) {
        pixelMover = new PixelMover("pixelMover", "Collects pixels and moves them", hardwareMap);
        this.telemetry = telemetry;
    }

    /**
     * Determine which button has been pressed or released.  When a button is pressed, invoke
     * the mechanism to perform the function assigned to the button.
     *
     * @param gamepad1
     * @param gamepad2
     * @param telemetry
     */
    public void execute(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (!aButtonPressed && gamepad1.a) {
            // User pressed the a button
            aButtonPressed = true;
            pixelMover.pickUpPixels();
        }
        if (aButtonPressed && !gamepad1.a) {
            // User released the a button
            aButtonPressed = false;
        }
        if (!bButtonPressed && gamepad1.b) {
            // User pressed the b button
            bButtonPressed = true;
            pixelMover.dropOffPixels();
        }
        if (bButtonPressed && !gamepad1.b) {
            // User released the b button
            bButtonPressed = false;
        }
    }

    public void dropOffPixels() {
        pixelMover.dropOffPixels();
    }
}
