package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;

public class PixelCollectorController implements Controller {

    private final Telemetry telemetry;
    PixelCollector leftPixelCollector;
    PixelCollector rightPixelCollector;
    private boolean dpad_down_was_pressed = false;
    private boolean dpad_right_was_pressed = false;
    private boolean gamepad_a_was_pressed = false;
    private boolean gamepad_b_was_pressed = false;

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
        if (!dpad_down_was_pressed && gamepad2.dpad_down) {
            telemetry.addLine("[LEFT PC] Set to collecting");
            dpad_down_was_pressed = true;
            // Toggle collection on or off
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.COLLECTING);
        } else if (!dpad_right_was_pressed && gamepad2.dpad_right) {
            telemetry.addLine("[LEFT PC] Set to depositing");
            dpad_right_was_pressed = true;
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
            // Toggle depositing on or off
        } else if (!gamepad2.dpad_down && !gamepad2.dpad_right && (dpad_down_was_pressed || dpad_right_was_pressed)) {
            telemetry.addLine("[LEFT PC] Set to closed");
            dpad_down_was_pressed = false;
            dpad_right_was_pressed = false;
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        }

        // Right pixel collector
        if (!gamepad_a_was_pressed && gamepad2.a) {
            telemetry.addLine("[RIGHT PC] Set to collecting");
            gamepad_a_was_pressed = true;
            // Toggle collection on or off
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.COLLECTING);
        } else if (!gamepad_b_was_pressed && gamepad2.b) {
            telemetry.addLine("[LEFT PC] Set to depositing");
            gamepad_b_was_pressed = true;
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
            // Toggle depositing on or off
        } else if (!gamepad2.a && !gamepad2.b && (gamepad_a_was_pressed || gamepad_b_was_pressed)) {
            telemetry.addLine("[LEFT PC] Set to closed");
            gamepad_a_was_pressed = false;
            gamepad_b_was_pressed = false;
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.CLOSED);
        }

        leftPixelCollector.update();
        rightPixelCollector.update();
    }
}
