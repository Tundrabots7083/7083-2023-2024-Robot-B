package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;

public class PixelCollectorController implements Controller {



    PixelCollector leftPixelCollector;
    PixelCollector rightPixelCollector;

    private final Telemetry telemetry;


    public PixelCollectorController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.rightPixelCollector = new PixelCollector("collectorRight", "Right pixel collector", hardwareMap, telemetry, false);
        this.leftPixelCollector = new PixelCollector("collectorLeft", "Left pixel collector", hardwareMap, telemetry, true);

        this.telemetry = telemetry;
    }

    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Gamepad 2 will control both pixel collectors
        // dpad down and dpad right will control the left pixel collector's state
        // a and b will be used for the right pixel collector's state

        // Left pixel collector
        if (gamepad2.dpad_down) {
            // Toggle collection on or off
            leftPixelCollector.toggleState(false);
        } else if (gamepad2.dpad_right) {
            // Toggle depositing on or off
            leftPixelCollector.toggleState(true);
        }

        // Right pixel collector
        if (gamepad2.a) {
            // Toggle collection on or off
            rightPixelCollector.toggleState(false);
        } else if (gamepad2.b) {
            // Toggle depositing on or off
            rightPixelCollector.toggleState(true);
        }
    }
}
