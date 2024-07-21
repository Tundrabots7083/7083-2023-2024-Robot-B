package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;

/**
 * Command to control one of the two pixel collectors on the robot. There will be two pixel
 * collector objects created for the robot.
 */
@Config
public class PixelCollectorCommand extends CommandBaseEx {
    public static long SPINNER_DELAY = 250;
    public static long FLAP_DELAY = 250;

    private final PixelCollector pixelCollector;
    private final Gamepad gamepad2;

    private ElapsedTime spinnerTimer = new ElapsedTime();
    private ElapsedTime flapTimer = new ElapsedTime();

    /**
     * Instantiates a pixel collector command.
     *
     * @param pixelCollector the pixel collector to control.
     * @param gamepad1       Gamepad1
     * @param gamepad2       Gamepad2
     */
    public PixelCollectorCommand(PixelCollector pixelCollector, Gamepad gamepad1, Gamepad gamepad2) {
        this.pixelCollector = pixelCollector;
        this.gamepad2 = gamepad2;

        addRequirements(pixelCollector);
    }

    /**
     * Runs the command. The controls used vary on whether the pixel collector is on the left or
     * right side.
     */
    @Override
    public void execute() {
        if (pixelCollector.getLocation() == PixelCollector.Location.LEFT) {
            setState(gamepad2.dpad_down, gamepad2.dpad_right);
        } else {
            setState(gamepad2.cross, gamepad2.circle);
        }
    }

    /**
     * Sets the flap position and spinner power for the pixel collector.
     *
     * @param intakePressed             the intake button is currently pressed.
     * @param depositingPressed         the depositing button is currently pressed.
     */
    private void setState(boolean intakePressed, boolean depositingPressed) {
        Telemetry telemetry = MyRobot.getInstance().telemetry;

        // Determine the target state based on the keys pressed
        if (intakePressed) {
            // Open the flap door, wait for FLAP_DELAY, and then turn on the spinner
            if (pixelCollector.getFlapPosition() != PixelCollector.FLAP_OPENED_POSITION) {
                pixelCollector.setFlapPosition(PixelCollector.FLAP_OPENED_POSITION);
                flapTimer.reset();
                telemetry.addData("[PC " + pixelCollector.getLocation() + "] state", "opening flap");
            } else if (pixelCollector.getSpinnerPower() != PixelCollector.SPINNER_COLLECTING_POWER) {
                if (flapTimer.milliseconds() > FLAP_DELAY) {
                    pixelCollector.setSpinnerPower(PixelCollector.SPINNER_COLLECTING_POWER);
                    telemetry.addData("[PC " + pixelCollector.getLocation() + "] state", "collecting");
                }
            }
        } else if (depositingPressed) {
            // Open the flap door, wait for FLAP_DELAY, and then turn on the spinner
            if (pixelCollector.getFlapPosition() != PixelCollector.FLAP_OPENED_POSITION) {
                pixelCollector.setFlapPosition(PixelCollector.FLAP_OPENED_POSITION);
                flapTimer.reset();
                telemetry.addData("[PC " + pixelCollector.getLocation() + "] state", "opening flap");
            } else if (pixelCollector.getSpinnerPower() != PixelCollector.SPINNER_DEPOSITING_POWER) {
                if (flapTimer.milliseconds() > FLAP_DELAY) {
                    pixelCollector.setSpinnerPower(PixelCollector.SPINNER_DEPOSITING_POWER);
                    telemetry.addData("[PC " + pixelCollector.getLocation() + "] state", "depositing");
                }
            }
        } else {
            // Turn off the spinner, wait for SPINNER_DELAY, and then close the flap
            if (pixelCollector.getSpinnerPower() != PixelCollector.SPINNER_OFF_POWER) {
                pixelCollector.setSpinnerPower(PixelCollector.SPINNER_OFF_POWER);
                spinnerTimer.reset();
                telemetry.addData("[PC " + pixelCollector.getLocation() + "] state", "stopping spinner");
            } else if (pixelCollector.getFlapPosition() != PixelCollector.FLAP_CLOSED_POSITION) {
                if (spinnerTimer.milliseconds() > SPINNER_DELAY) {
                    pixelCollector.setFlapPosition(PixelCollector.FLAP_CLOSED_POSITION);
                    telemetry.addData("[PC " + pixelCollector.getLocation() + "] state", "closing flap");
                    flapTimer.reset();
                }
            } else if (pixelCollector.getFlapPosition() == PixelCollector.FLAP_CLOSED_POSITION &&
                       pixelCollector.getSpinnerPower() == PixelCollector.SPINNER_OFF_POWER &&
                       flapTimer.milliseconds() > FLAP_DELAY &&
                       spinnerTimer.milliseconds() > SPINNER_DELAY) {
                telemetry.addData("[PC " + pixelCollector.getLocation() + "] state", "stopped");
            }
        }
    }
}
