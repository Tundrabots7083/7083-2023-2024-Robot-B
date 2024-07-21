package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

/**
 * Command to control the lift behavior.
 */
public class LiftCommand extends CommandBaseEx {
    private final Lift lift;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private boolean manualOverride = false;

    /**
     * Instantiates an instance of the lift command. The lift command takes input from the
     * gamepads and translates them to values to change the position of the lift.
     *
     * @param lift     the lift the command controls
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    public LiftCommand(Lift lift, Gamepad gamepad1, Gamepad gamepad2) {
        this.lift = lift;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        addRequirements(lift);
    }

    /**
     * Command logic that runs each robot command cycle.
     */
    @Override
    public void execute() {
        Telemetry telemetry = MyRobot.getInstance().telemetry;

        // Adjust the lift position based on the key pressed
        if (gamepad1.dpad_down) {
            lift.setPosition(Lift.INTAKE_POSITION);
            manualOverride = false;
        } else if (gamepad1.dpad_left) {
            lift.setPosition(Lift.SCORE_LOW_POSITION);
            manualOverride = false;
        } else if (gamepad1.dpad_right) {
            lift.setPosition(Lift.SCORE_MEDIUM_POSITION);
            manualOverride = false;
        } else if (gamepad1.dpad_up) {
            lift.setPosition(Lift.SCORE_HIGH_POSITION);
            manualOverride = false;
        } else if (gamepad2.triangle) {
            lift.setPosition(Lift.INTAKE_POSITION);
            manualOverride = false;
        } else if (gamepad1.square) {
            lift.setPosition(Lift.HANG_START_POSITION);
            manualOverride = false;
        } else if (gamepad1.triangle) {
            lift.setPosition(Lift.HANG_END_POSITION);
            manualOverride = false;
        }

        // Manual override. This is used in cases where a "smack of doom" scenario occurs, where
        // one or both of the motor encoders cannot be accessed and the power applied to the motor
        // results in undesirable behavior, such as damaging the field.
        if (manualOverride || (gamepad1.right_trigger != 0.0 || gamepad1.left_trigger != 0.0)) {
            manualOverride = true;
            if (gamepad1.left_trigger > 0.0) {
                lift.setPower(gamepad1.left_trigger, gamepad1.left_trigger);
                telemetry.addData("[LIFT] power override", gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.0) {
                lift.setPower(-gamepad1.right_trigger, -gamepad1.right_trigger);
                telemetry.addData("[LIFT] power override", -gamepad1.right_trigger);
            } else {
                lift.setPower(0, 0);
                telemetry.addData("[ARM] power override", 0.0);
            }
            return;
        }

        lift.execute();
    }
}
