package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Controller is the base interface for all controls on the robot.
 */
public interface Controller {
    void execute(Gamepad gamepad1, Gamepad gamepad2);
}
