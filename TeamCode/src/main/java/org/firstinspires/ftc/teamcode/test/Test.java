package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test is the base class for all tests for the Robot.
 */
public abstract class Test {
    private final String description;

    Test(String description) {
        this.description = description;
    }

    public String getDescription() {
        return description;
    }

    public abstract void run(Gamepad gamepad1, Telemetry telemetry);
}