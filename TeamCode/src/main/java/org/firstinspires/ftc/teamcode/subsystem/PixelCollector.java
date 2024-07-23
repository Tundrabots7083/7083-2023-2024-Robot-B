package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx;

/**
 * Class for controlling a single pixel collector.
 * A pixel collector is defined as a flap servo + a spinner servo.
 * Robot B has two pixel collectors
 */
@Config
public class PixelCollector extends SubsystemBaseEx {
    public static double SPINNER_COLLECT_POWER = -0.3;
    public static double SPINNER_DEPOSIT_POWER = 0.09;
    public static double SPINNER_OFF_POWER = 0.0;
    public static double FLAP_OPENED_POSITION = 0.0;
    public static double FLAP_CLOSED_POSITION = 0.65;

    public static long SPINNER_DELAY = 250;
    public static long DEPOSIT_PIXEL_TIME = 1750;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 180;

    private final Telemetry telemetry;
    private final CRServoEx spinner;
    private final ServoEx flap;

    private long spinnerDelayTime;
    private PixelCollectorState state;

    /**
     * Creates an instance of a pixel collector.
     *
     * @param location    location of the pixel collector on the robot
     * @param hardwareMap the mapping of all hardware on the bot.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public PixelCollector(Location location, HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        String deviceName = location == Location.LEFT ? "collectorLeft" : "collectorRight";
        spinner = new CRServoEx(hardwareMap, deviceName + "Spinner");
        flap = new SimpleServo(hardwareMap, deviceName + "Flap", MIN_ANGLE, MAX_ANGLE);

        // The left pixel collector is reversed; the right pixel collector is not
        if (location == Location.LEFT) {
            flap.setInverted(true);
            spinner.setInverted(true);
        }

        // Initialize the pixel collector state
        state = PixelCollectorState.IDLE;
        flap.setPosition(FLAP_CLOSED_POSITION);
        spinner.set(SPINNER_OFF_POWER);

        spinnerDelayTime = System.currentTimeMillis();
    }

    /**
     * Sets the state for the pixel collector.
     *
     * @param state the state for the pixel collector.
     */
    public void setState(PixelCollectorState state) {
        if (this.state != state) {
            this.state = state;
            spinnerDelayTime = System.currentTimeMillis() + SPINNER_DELAY;
        }
    }

    /**
     * Update the position of the flap and the power for the spinner for the pixel collector.
     */
    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        switch (state) {
            case IDLE:
                // Turn off the power to the spinner, and wait for it to stop before closing the
                // pixel collector flap
                spinner.set(SPINNER_OFF_POWER);
                if (currentTime > spinnerDelayTime) {
                    flap.setPosition(FLAP_CLOSED_POSITION);
                }
                break;
            case DEPOSITING:
                // Open the pixel collector flap, and wait for it to be fully opened before starting
                // the spinner
                flap.setPosition(FLAP_OPENED_POSITION);
                if (currentTime > spinnerDelayTime) {
                    spinner.set(SPINNER_DEPOSIT_POWER);
                }
                break;
            case COLLECTING:
                // Open the pixel collector flap, and wait for it to be fully opened before starting
                // the spinner
                flap.setPosition(FLAP_OPENED_POSITION);
                if (currentTime > spinnerDelayTime) {
                    spinner.set(SPINNER_COLLECT_POWER);
                }
                break;
        }
    }

    /**
     * Gets an action to deposit a pixel from the pixel collector.
     *
     * @return the action to deposit a pixel from the pixel collector.
     */
    public Action depositPixel() {
        return new DepositPixelAction(this);
    }

    /**
     * The state for the pixel collector.
     */
    public enum PixelCollectorState {
        /**
         * Collecting pixels from the field
         */
        COLLECTING,
        /**
         * Depositing pixels on the field
         */
        DEPOSITING,
        /**
         * The spinner is off and the flap door is closed
         */
        IDLE,
    }

    /**
     * Location of the pixel collector on the robot
     */
    public enum Location {
        LEFT,
        RIGHT
    }

    /**
     * Action to deposit a pixel from the given collector. This will open the flap door, turn
     * on the spinner, wait for the pixel to be deposited, turn off the spinner, and then close
     * the flap door.
     */
    private static class DepositPixelAction implements Action {
        private final PixelCollector pixelCollector;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean isInitialized = false;

        /**
         * Instantiates an action to deposit a pixel.
         *
         * @param pixelCollector the pixel collector from which to deposit a pixel.
         */
        public DepositPixelAction(PixelCollector pixelCollector) {
            this.pixelCollector = pixelCollector;
            pixelCollector.setState(PixelCollectorState.DEPOSITING);
        }

        /**
         * Initialize the action
         */
        private void initialize() {
            pixelCollector.setState(PixelCollectorState.DEPOSITING);
            pixelCollector.execute();
            timer.reset();
        }

        /**
         * Deposits a pixel from the collector and then turns closes the pixel collector.
         *
         * @param telemetryPacket telemetry used to display output
         * @return <code>false</code> if the pixel has been deposited;
         * <code>true</code> if the action needs to continue running to deposit the pixel.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isInitialized) {
                initialize();
                isInitialized = true;
            }

            final boolean isFinished;
            switch (pixelCollector.state) {
                case DEPOSITING:
                    if (timer.milliseconds() > DEPOSIT_PIXEL_TIME) {
                        pixelCollector.setState(PixelCollectorState.IDLE);
                        timer.reset();
                    }
                    isFinished = false;
                    break;
                case IDLE:
                    isFinished = timer.milliseconds() > SPINNER_DELAY;
                    break;
                default:
                    isFinished = true;
            }

            pixelCollector.execute();

            return !isFinished;
        }
    }
}
