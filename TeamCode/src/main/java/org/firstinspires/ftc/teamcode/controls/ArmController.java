package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;

/**
 * Uses the gamepad controller to set the position of the arm and container servo.
 */
public class ArmController implements Controller {
    private final Arm arm = new Arm("arm", "scoring arm");

    private boolean manualControl = false;

    /**
     * Initializes the arm hardware.
     * @param hardwareMap the hardware map for the robot.
     */
    @Override
    public void init(HardwareMap hardwareMap) {
        arm.init(hardwareMap);
    }

    public void start() {
        arm.setTarget(Arm.Position.Start);
    }

    /**
     * Sets the position of the arm, and calls the arm to update the position.
     * @param gamepad the gamepad controller for the arm.
     * @param telemetry the telemetry used to output data for the arm.
     */
    @Override
    public void execute(Gamepad gamepad, Telemetry telemetry) {
        // Manual override of controls
        if (manualControl || gamepad.right_trigger != 0.0 || gamepad.left_trigger != 0.0) {
            manualControl = true;

            if (gamepad.left_trigger > 0.0) {
                // arm.disableRunToPosition();
                arm.setArmPower(gamepad.left_trigger);
                telemetry.addData("[ARM] Power", gamepad.left_trigger);
            }
            else if (gamepad.right_trigger > 0.0) {
                // arm.disableRunToPosition();
                arm.setArmPower(-gamepad.right_trigger);
                telemetry.addData("[ARM] Power", -gamepad.right_trigger);
            } else {
                arm.setArmPower(0.0);
                telemetry.addData("[ARM] Power", 0.0);
            }
            telemetry.update();
            return ;
        }

        // Automatic update of controls
        if (gamepad.dpad_down) {
            arm.setTarget(Arm.Position.Intake);
        } else if (gamepad.dpad_left) {
            arm.setTarget(Arm.Position.ScoreLow);
        } else if (gamepad.dpad_right) {
            arm.setTarget(Arm.Position.ScoreMedium);
        } else if (gamepad.dpad_up) {
            arm.setTarget(Arm.Position.ScoreHigh);
        } else if (gamepad.right_bumper) {
            arm.setTarget(Arm.Position.Hang);
        }

        arm.update();

        telemetry.addData("[ARM] At Target", arm.isAtTarget());
        telemetry.addData("[ARM] Arm Target Pos", arm.getTargetArmPosition());
        telemetry.addData("[ARM] Arm Current Pos", arm.getCurrentArmPosition());
        // telemetry.addData("[ARM] Lift Target Pos", arm.getTargetLiftPosition());
        // telemetry.addData("[ARM] Lift Current Pos", arm.getCurrentLiftPosition());
        telemetry.addData("[ARM] Servo Target Pos", arm.getTargetContainerServoPosition());
        telemetry.addData("[ARM] Servo Current Pos", arm.getCurrentContainerServoPosition());
        telemetry.update();

        /*

        if (gamepad.left_trigger > 0.0) {
            // arm.disableRunToPosition();
            arm.setArmPower(gamepad.left_trigger);
            telemetry.addData("[ARM] Power", gamepad.left_trigger);
        }
        else if (gamepad.right_trigger > 0.0) {
            // arm.disableRunToPosition();
            arm.setArmPower(-gamepad.right_trigger);
            telemetry.addData("[ARM] Power", -gamepad.right_trigger);
        } else {
            arm.setArmPower(0.0);
            telemetry.addData("[ARM] Power", 0.0);
        }
        telemetry.update();
         */
    }
}
