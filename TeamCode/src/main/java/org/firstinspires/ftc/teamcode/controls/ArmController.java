package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;

/**
 * Uses the gamepad controller to set the position of the arm and container servo.
 */
public class ArmController implements Controller {
    private final Arm arm;
    private Telemetry telemetry;
    private boolean manualOverride = false;

    /**
     * Initializes the arm hardware.
     * @param hardwareMap the hardware map for the robot.
     */
    public ArmController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        arm = new Arm("arm", "scoring arm", hardwareMap);
    }

    public void start() {
        arm.setTarget(Arm.Position.Start);
    }

    public void setToLauncheDrone() {
        arm.setTarget(Arm.Position.LaunchDrone);
        while (!arm.isAtTarget()) {
            arm.update();
        }
    }

    /**
     * Sets the position of the arm, and calls the arm to update the position.
     * @param gamepad1 the gamepad1 controller for the arm.
     * @param gamepad1 the gamepad2 controller for the arm.
     * @param telemetry the telemetry used to output data for the arm.
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        // Automatic update of controls
        if (gamepad1.dpad_down) {
            arm.setTarget(Arm.Position.Intake);
            manualOverride = false;
        } else if (gamepad1.dpad_left) {
            arm.setTarget(Arm.Position.ScoreLow);
            manualOverride = false;
        } else if (gamepad1.dpad_right) {
            arm.setTarget(Arm.Position.ScoreMedium);
            manualOverride = false;
        } else if (gamepad1.dpad_up) {
            arm.setTarget(Arm.Position.ScoreHigh);
            manualOverride = false;
        } else if (gamepad1.x) {
            arm.setTarget(Arm.Position.LaunchDrone);
            manualOverride = false;
        } else if (gamepad1.y) {
            arm.setTarget(Arm.Position.Hang);
            manualOverride = false;
        }

        // Manual override of controls
        if (manualOverride || (gamepad1.right_trigger != 0.0 || gamepad1.left_trigger != 0.0)) {
            manualOverride = true;
            if (gamepad1.left_trigger > 0.0) {
                // arm.disableRunToPosition();
                arm.setArmPower(gamepad1.left_trigger);
                telemetry.addData("[ARM] Power", gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0.0) {
                // arm.disableRunToPosition();
                arm.setArmPower(-gamepad1.right_trigger);
                telemetry.addData("[ARM] Power", -gamepad1.right_trigger);
            } else {
                arm.setArmPower(0.0);
                telemetry.addData("[ARM] Power", 0.0);
            }
            return ;
        }

        arm.update();

        telemetry.addData("[ARM] At Target", arm.isAtTarget());
        telemetry.addData("[ARM] Arm Target Pos", arm.getTargetArmPosition());
        telemetry.addData("[ARM] Arm Current Pos", arm.getCurrentArmPosition());
        // telemetry.addData("[ARM] Lift Target Pos", arm.getTargetLiftPosition());
        // telemetry.addData("[ARM] Lift Current Pos", arm.getCurrentLiftPosition());
        telemetry.addData("[ARM] Servo Target Pos", arm.getTargetContainerServoPosition());
        telemetry.addData("[ARM] Servo Current Pos", arm.getCurrentContainerServoPosition());

        /*

        if (gamepad1.left_trigger > 0.0) {
            // arm.disableRunToPosition();
            arm.setArmPower(gamepad1.left_trigger);
            telemetry.addData("[ARM] Power", gamepad1.left_trigger);
        }
        else if (gamepad1.right_trigger > 0.0) {
            // arm.disableRunToPosition();
            arm.setArmPower(-gamepad1.right_trigger);
            telemetry.addData("[ARM] Power", -gamepad1.right_trigger);
        } else {
            arm.setArmPower(0.0);
            telemetry.addData("[ARM] Power", 0.0);
        }
         */
    }

    Arm getArm() {
        return arm;
    }
}
