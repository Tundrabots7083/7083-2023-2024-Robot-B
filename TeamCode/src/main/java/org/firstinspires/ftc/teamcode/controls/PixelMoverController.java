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
    }

    /**
     * Toggles the pixel mover between being stopped and moving based on whether the
     * A button has been pressed or the B button has been pressed.
     *
     * Pressing the A button toggles the pixel mover between being stopped and moving forward.
     *   Forward movement results in pixels being swept into the robot's pixel container.
     * Pressing the B button toggles the pixel mover between being stopped and moving in reverse.
     *   Reverse movement results in pixels being pushed out of the robot's pixel container.
     *
     * @param gamepad
     * @param telemetry
     */
    public void toggleState(Gamepad gamepad, Telemetry telemetry) {
        if (!aButtonPressed && gamepad.a) {
            // User pressed the a button
            aButtonPressed = true;
            // Tell pixel mover to toggle itself between being stopped and moving forward
            pixelMover.toggleForward();
        }
        if (aButtonPressed && !gamepad.a) {
            // User released the a button
            aButtonPressed = false;
        }
        if (!bButtonPressed && gamepad.b) {
            // User pressed the b button
            bButtonPressed = true;
            // Tell pixel mover to toggle itself between being stopped and moving in reverse
            pixelMover.toggleReversed();
        }
        if (bButtonPressed && !gamepad.b) {
            // User released the b button
            bButtonPressed = false;
        }
    }
}
