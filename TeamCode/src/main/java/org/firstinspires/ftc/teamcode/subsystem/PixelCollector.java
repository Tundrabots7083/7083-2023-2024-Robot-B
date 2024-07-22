package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private final Telemetry telemetry;
    private final CRServo spinner;
    private final Servo flap;
    private long spinnerDelayTime;
    private PixelCollectorState state;

    /**
     * Creates an instance of a pixel collector.
     *
     * @param deviceName       the device name for the pixel collector.
     * @param description      the description of the pixel collector.
     * @param hardwareMap      the mapping of all hardware on the bot.
     * @param telemetry        the telemetry used to display data on the driver station.
     * @param reverseFlapServo whether the servo runs forward or reverse.
     */
    public PixelCollector(String deviceName, String description, HardwareMap hardwareMap, Telemetry telemetry, boolean reverseFlapServo) {
        this.spinner = hardwareMap.get(CRServo.class, deviceName + "Spinner");
        this.flap = hardwareMap.get(Servo.class, deviceName + "Flap");
        this.telemetry = telemetry;

        if (reverseFlapServo) {
            this.flap.setDirection(Servo.Direction.REVERSE);
            this.spinner.setDirection(CRServo.Direction.REVERSE);
        }

        this.state = PixelCollectorState.CLOSED;
        this.flap.setPosition(FLAP_CLOSED_POSITION);
        this.spinner.setPower(SPINNER_OFF_POWER);

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
            case CLOSED:
                spinner.setPower(SPINNER_OFF_POWER);
                if (currentTime > spinnerDelayTime) {
                    flap.setPosition(FLAP_CLOSED_POSITION);
                }
                break;
            case DEPOSITING:
                flap.setPosition(FLAP_OPENED_POSITION);
                if (currentTime > spinnerDelayTime) {
                    spinner.setPower(SPINNER_DEPOSIT_POWER);
                }
                break;
            case COLLECTING:
                flap.setPosition(FLAP_OPENED_POSITION);
                if (currentTime > spinnerDelayTime) {
                    spinner.setPower(SPINNER_COLLECT_POWER);
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
        /** Collecting pixels from the field */
        COLLECTING,
        /** Depositing pixels on the field */
        DEPOSITING,
        /** The spinner is off and the flap door is closed */
        CLOSED,
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
                        pixelCollector.setState(PixelCollectorState.CLOSED);
                        timer.reset();
                    }
                    isFinished = false;
                    break;
                case CLOSED:
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
