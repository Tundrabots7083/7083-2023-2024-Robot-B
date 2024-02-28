package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;

public class PixelCollectorTest extends Test {
    private final PixelCollector pixelCollector;

    public PixelCollectorTest(PixelCollector pixelCollector) {
        super("Pixel Collector Test");

        this.pixelCollector = pixelCollector;
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a) {
            pixelCollector.setSpinnerPower(PixelCollector.State.DEPOSITING.spinnerPower);
        } else if (gamepad1.b) {
            pixelCollector.setSpinnerPower(PixelCollector.State.CLOSED.spinnerPower);
        } else if (gamepad1.x) {
            pixelCollector.setFlapPosition(PixelCollector.State.DEPOSITING.flapPosition);
        } else if (gamepad1.y) {
            pixelCollector.setFlapPosition(PixelCollector.State.CLOSED.flapPosition);
        }
    }
}
