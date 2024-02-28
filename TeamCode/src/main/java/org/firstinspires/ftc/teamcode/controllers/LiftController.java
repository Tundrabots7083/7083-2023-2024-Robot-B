package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

/**
 * Uses the gamepad controller to set the position of the arm and container servo.
 */
public class LiftController implements Controller {
    private final Lift lift;
    private final Telemetry telemetry;
    private boolean manualOverride = false;

    /**
     * Initializes the lift controller.
     *
     * @param lift      the lift being controlled.
     * @param telemetry the telemetry used to write data to the drive station.
     */
    public LiftController(Lift lift, Telemetry telemetry) {
        this.lift = lift;
        this.telemetry = telemetry;
    }

    /**
     * Initializes the lift to the starting position.
     */
    public void start() {
        lift.setTarget(Lift.Position.INTAKE);
        lift.update();
    }

    /**
     * Sets the position of the arm, and calls the arm to update the position.
     *
     * @param gamepad1 the gamepad1 controller for the arm.
     * @param gamepad2 the gamepad2 controller for the arm.
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Automatic update of controls
        if (gamepad1.dpad_down) {
            lift.setTarget(Lift.Position.INTAKE);
            manualOverride = false;
        } else if (gamepad1.dpad_left) {
            lift.setTarget(Lift.Position.SCORE_LOW);
            manualOverride = false;
        } else if (gamepad1.dpad_right) {
            lift.setTarget(Lift.Position.SCORE_MEDIUM);
            manualOverride = false;
        } else if (gamepad1.dpad_up) {
            lift.setTarget(Lift.Position.SCORE_HIGH);
            manualOverride = false;
        } else if (gamepad2.y) {
            lift.setTarget(Lift.Position.LAUNCH_DRONE);
            manualOverride = false;
        } else if (gamepad1.x) {
            lift.setTarget(Lift.Position.HANG_START);
            manualOverride = false;
        } else if (gamepad1.y) {
            lift.setTarget(Lift.Position.HANG_END);
            manualOverride = false;
        }

        if (manualOverride || (gamepad1.right_trigger != 0.0 || gamepad1.left_trigger != 0.0)) {
            manualOverride = true;
            if (gamepad1.left_trigger > 0.0) {
                lift.overrideLiftPower(gamepad1.left_trigger);
                telemetry.addData("[LIFT] Power", gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.0) {
                lift.overrideLiftPower(-gamepad1.right_trigger);
                telemetry.addData("[LIFT] Power", -gamepad1.right_trigger);
            } else {
                lift.overrideLiftPower(0.0);
                telemetry.addData("[ARM] Power", 0.0);
            }
            return;
        }

        lift.update();
    }
}
