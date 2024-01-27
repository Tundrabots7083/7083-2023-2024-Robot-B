package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
     * Initializes the arm hardware.
     * @param hardwareMap the hardware map for the robot.
     */
    public LiftController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        lift = new Lift(hardwareMap);
    }

    public void start() {
        lift.setTarget(Lift.Position.Intake);
        lift.update();
    }

    /**
     * Sets the position of the arm, and calls the arm to update the position.
     * @param gamepad1 the gamepad1 controller for the arm.
     * @param gamepad2 the gamepad2 controller for the arm.
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Automatic update of controls
        if (gamepad1.dpad_down) {
            lift.setTarget(Lift.Position.Intake);
            manualOverride = false;
        } else if (gamepad1.dpad_left) {
            lift.setTarget(Lift.Position.ScoreLow);
            manualOverride = false;
        } else if (gamepad1.dpad_right) {
            lift.setTarget(Lift.Position.ScoreMedium);
            manualOverride = false;
        } else if (gamepad1.dpad_up) {
            lift.setTarget(Lift.Position.ScoreHigh);
            manualOverride = false;
        } else if (gamepad1.x) {
            lift.setTarget(Lift.Position.LaunchDrone);
            manualOverride = false;
        } else if (gamepad1.y) {
            lift.setTarget(Lift.Position.Hang);
            manualOverride = false;
        }

        // Manual override of controls
//        if (manualOverride || (gamepad1.right_trigger != 0.0 || gamepad1.left_trigger != 0.0)) {
//            manualOverride = true;
//            if (gamepad1.left_trigger > 0.0) {
//                lift.setArmPower(gamepad1.left_trigger);
//                telemetry.addData("[ARM] Power", gamepad1.left_trigger);
//            }
//            else if (gamepad1.right_trigger > 0.0) {
//                lift.setArmPower(-gamepad1.right_trigger);
//                telemetry.addData("[ARM] Power", -gamepad1.right_trigger);
//            } else {
//                lift.setArmPower(0.0);
//                telemetry.addData("[ARM] Power", 0.0);
//            }
//            return ;
//        }

        lift.update();
    }
}
