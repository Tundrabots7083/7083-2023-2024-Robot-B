package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;

public class PixelCollectorController implements Controller {

    private final Telemetry telemetry;
    private final Gamepad previousGamepad2 = new Gamepad();
    PixelCollector leftPixelCollector;
    PixelCollector rightPixelCollector;

    /**
     * Creates a new controller to manage the left and right pixel collectors.
     *
     * @param leftPixelCollector  the left pixel collector
     * @param rightPixelCollector the right pixel collector
     * @param telemetry           the telemetry used to display data on the driver station
     */
    public PixelCollectorController(PixelCollector leftPixelCollector, PixelCollector rightPixelCollector, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.leftPixelCollector = leftPixelCollector;
        this.rightPixelCollector = rightPixelCollector;
    }

    private void setState(PixelCollector pixelCollector, boolean intakePressed, boolean previousIntakePressed, boolean depositingPressed, boolean previousDepositingPressed) {
        String collectorName = pixelCollector == leftPixelCollector ? "[LEFT PC] " : "[RIGHT PC] ";

        if (!previousIntakePressed && intakePressed) {
            telemetry.addLine(collectorName + "Set to collecting");
            // Toggle collection on or off
            pixelCollector.setState(PixelCollector.PixelCollectorState.COLLECTING);
        } else if (!previousDepositingPressed && depositingPressed) {
            telemetry.addLine(collectorName + "Set to depositing");
            pixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
            // Toggle depositing on or off
        } else if (!intakePressed && !depositingPressed && (previousIntakePressed || previousDepositingPressed)) {
            telemetry.addLine(collectorName + "Set to closed");
            pixelCollector.setState(PixelCollector.PixelCollectorState.IDLE);
        }
    }

    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Gamepad 2 will control both pixel collectors
        // dpad down and dpad right will control the left pixel collector's state
        // a and b will be used for the right pixel collector's state

        setState(leftPixelCollector, gamepad2.dpad_down, previousGamepad2.dpad_down, gamepad2.dpad_right, previousGamepad2.dpad_right);
        setState(rightPixelCollector, gamepad2.a, previousGamepad2.a, gamepad2.b, previousGamepad2.b);

        leftPixelCollector.execute();
        rightPixelCollector.execute();

        previousGamepad2.copy(gamepad2);
    }
}
