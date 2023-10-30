package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PixelMover;

public class PixelMoverController implements Controller {
    private boolean aButtonPressed = false;
    private boolean bButtonPressed = false;

    private PixelMover pixelMover;

    public PixelMoverController() {}

    @Override
    public void init(HardwareMap hardwareMap) {
        pixelMover = new PixelMover("pixelMover", "Collects pixels and moves them");
        pixelMover.init(hardwareMap);
    }


    /**
     * Determine which button has been pressed or released.  When a button is pressed, invoke
     * the mechanism to perform the function assigned to the button.
     *
     * @param gamepad
     * @param telemetry
     */
    public void execute(Gamepad gamepad, Telemetry telemetry) {
        if (!aButtonPressed && gamepad.a) {
            // User pressed the a button
            aButtonPressed = true;
            pixelMover.pickUpPixels();
        }
        if (aButtonPressed && !gamepad.a) {
            // User released the a button
            aButtonPressed = false;
        }
        if (!bButtonPressed && gamepad.b) {
            // User pressed the b button
            bButtonPressed = true;
            pixelMover.dropOffPixels();
        }
        if (bButtonPressed && !gamepad.b) {
            // User released the b button
            bButtonPressed = false;
        }
    }
}
