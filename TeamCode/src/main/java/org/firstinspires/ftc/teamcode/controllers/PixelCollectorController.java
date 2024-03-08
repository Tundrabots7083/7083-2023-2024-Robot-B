package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

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
     *
     * @param leftPixelCollector  the left pixel collector.
     * @param rightPixelCollector the right pixel collector.
     * @param telemetry           the telemetry used to display output on the driver station.
     */
    public PixelCollectorController(PixelCollector leftPixelCollector, PixelCollector rightPixelCollector, Telemetry telemetry) {
        this.rightPixelCollector = leftPixelCollector;
        this.leftPixelCollector = rightPixelCollector;

        this.telemetry = telemetry;
    }

    /**
     * Updates the pixel collector state based on the Gamepad buttons pressed.
     *
     * @param collector       PixelCollector being updated
     * @param collect         the <i>collect</i> button is pressed
     * @param previousCollect the <i>collect</i> button was pressed on the previous loop
     * @param score           the <i>score</i> button is pressed
     * @param previousScore   the <i>score</i> button was pressed on the previous loop
     */
    private void update(PixelCollector collector, boolean collect, boolean previousCollect, boolean score, boolean previousScore) {
        String c = collector == leftPixelCollector ? "[PC Left] " : "[PC Right] ";
        if (!previousCollect && collect) {
            collector.setState(PixelCollector.State.COLLECTING);
            telemetry.addLine(c + "Set State to Collecting");
        } else if (!previousScore && score) {
            collector.setState(PixelCollector.State.DEPOSITING);
            telemetry.addLine(c + "Set State to Depositing");
        } else if (!collect && !score && (previousCollect || previousScore)) {
            leftPixelCollector.setState(PixelCollector.State.CLOSED);
            telemetry.addLine(c + "Set State to Closed");
        }
    }

    /**
     * Process input to the pixel collector controller each loop through the opmode.
     *
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Gamepad 2 will control both pixel collectors
        // dpad down and dpad right will control the left pixel collector's state
        // a and b will be used for the right pixel collector's state
        update(leftPixelCollector, gamepad2.dpad_down, previousGamepad2.dpad_down, gamepad2.dpad_right, previousGamepad2.dpad_right);
        update(rightPixelCollector, gamepad2.a, previousGamepad2.a, gamepad2.b, previousGamepad2.b);

        leftPixelCollector.update();
        rightPixelCollector.update();

        previousGamepad2.copy(gamepad2);
    }
}
