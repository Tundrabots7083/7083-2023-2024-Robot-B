package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.ScoringSubsystem;

/**
 * Uses the gamepad controller to set the position of the arm and container servo.
 */
public class LiftController implements Controller {
    private final ScoringSubsystem scoringSubsystem;
    private final Telemetry telemetry;
    private boolean manualOverride = false;

    /**
     * Initializes the arm hardware.
     *
     * @param scoringSubsystem      the lift to be controlled.
     * @param telemetry the telemetry used to display data on the driver station.
     */
    public LiftController(ScoringSubsystem scoringSubsystem, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.scoringSubsystem = scoringSubsystem;
    }

    public void start() {
        scoringSubsystem.setTarget(ScoringSubsystem.Position.INTAKE);
        scoringSubsystem.execute();
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
            scoringSubsystem.setTarget(ScoringSubsystem.Position.INTAKE);
            manualOverride = false;
        } else if (gamepad1.dpad_left) {
            scoringSubsystem.setTarget(ScoringSubsystem.Position.SCORE_LOW);
            manualOverride = false;
        } else if (gamepad1.dpad_right) {
            scoringSubsystem.setTarget(ScoringSubsystem.Position.SCORE_MEDIUM);
            manualOverride = false;
        } else if (gamepad1.dpad_up) {
            scoringSubsystem.setTarget(ScoringSubsystem.Position.SCORE_HIGH);
            manualOverride = false;
        } else if (gamepad2.y) {
            scoringSubsystem.setTarget(ScoringSubsystem.Position.LAUNCH_DRONE);
            manualOverride = false;
        } else if (gamepad1.x) {
            scoringSubsystem.setTarget(ScoringSubsystem.Position.HANG_START);
            manualOverride = false;
        } else if (gamepad1.y) {
            scoringSubsystem.setTarget(ScoringSubsystem.Position.HANG_END);
            manualOverride = false;
        }

        if (manualOverride || (gamepad1.right_trigger != 0.0 || gamepad1.left_trigger != 0.0)) {
            Robot robot = Robot.getInstance();
            manualOverride = true;
            if (gamepad1.left_trigger > 0.0) {
                robot.lift.overrideLiftPower(gamepad1.left_trigger);
                telemetry.addData("[LIFT] Power", gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.0) {
                robot.lift.overrideLiftPower(-gamepad1.right_trigger);
                telemetry.addData("[LIFT] Power", -gamepad1.right_trigger);
            } else {
                robot.lift.overrideLiftPower(0.0);
                telemetry.addData("[ARM] Power", 0.0);
            }
            return;
        }

        scoringSubsystem.execute();
    }
}
