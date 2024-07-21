package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.Arm;

/**
 * Uses the gamepad controller to set the position of the arm and container servo.
 */
public class ScoreSubsystemController implements Controller {
    private final Lift lift;
    private final Arm arm;
    private boolean manualOverride = false;

    /**
     * Initializes the arm hardware.
     *
     * @param lift      the lift to be controlled.
     */
    public ScoreSubsystemController(Lift lift, Arm arm) {
        this.lift = lift;
        this.arm = arm;
    }

    public void start() {
        lift.setPosition(Lift.INTAKE_POSITION);
        arm.setPosition(Arm.INTAKE_POSITION);
        lift.execute();
        arm.execute();
    }

    /**
     * Sets the position of the arm, and calls the arm to update the position.
     *
     * @param gamepad1 the gamepad1 controller for the arm.
     * @param gamepad2 the gamepad2 controller for the arm.
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        Telemetry telemetry = MyRobot.getInstance().telemetry;

        // Automatic update of controls
        if (gamepad1.dpad_down) {
            lift.setPosition(Lift.INTAKE_POSITION);
            arm.setPosition(Arm.INTAKE_POSITION);
            manualOverride = false;
        } else if (gamepad1.dpad_left) {
            lift.setPosition(Lift.SCORE_LOW_POSITION);
            arm.setPosition(Arm.SCORE_POSITION);
            manualOverride = false;
        } else if (gamepad1.dpad_right) {
            lift.setPosition(Lift.SCORE_MEDIUM_POSITION);
            arm.setPosition(Arm.SCORE_POSITION);
            manualOverride = false;
        } else if (gamepad1.dpad_up) {
            lift.setPosition(Lift.SCORE_HIGH_POSITION);
            arm.setPosition(Arm.SCORE_POSITION);
            manualOverride = false;
        } else if (gamepad2.y) {
            lift.setPosition(Lift.DRONE_LAUNCH_POSITION);
            arm.setPosition(Arm.INTAKE_POSITION);
            manualOverride = false;
        } else if (gamepad1.x) {
            lift.setPosition(Lift.HANG_START_POSITION);
            arm.setPosition(Arm.INTAKE_POSITION);
            manualOverride = false;
        } else if (gamepad1.y) {
            lift.setPosition(Lift.HANG_END_POSITION);
            arm.setPosition(Arm.INTAKE_POSITION);
            manualOverride = false;
        }

        if (manualOverride || (gamepad1.right_trigger != 0.0 || gamepad1.left_trigger != 0.0)) {
            manualOverride = true;
            if (gamepad1.left_trigger > 0.0) {
                lift.setPower(gamepad1.left_trigger, gamepad1.left_trigger);
                telemetry.addData("[LIFT] Power", gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.0) {
                lift.setPower(-gamepad1.right_trigger, -gamepad1.right_trigger);
                telemetry.addData("[LIFT] Power", -gamepad1.right_trigger);
            } else {
                lift.setPower(0.0, 0.0);
                telemetry.addData("[ARM] Power", 0.0);
            }
            return;
        }

        lift.execute();
    }
}
