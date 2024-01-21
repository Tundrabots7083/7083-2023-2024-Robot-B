package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Controller is the base interface for all controls on the robot.
 */
public interface Controller {
    public void execute(Gamepad gamepad1, Gamepad gamepad2);
}
