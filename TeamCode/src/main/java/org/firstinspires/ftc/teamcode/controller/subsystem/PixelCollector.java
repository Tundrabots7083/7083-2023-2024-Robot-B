package org.firstinspires.ftc.teamcode.controller.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.CommandBaseEx;
import org.firstinspires.ftc.teamcode.command.CommandEx;

/**
 * Pixel collector that is responsible for picking up pixels and depositing pixels. There will be
 * two pixel collectors on the robot.
 */
@Config
public class PixelCollector extends Subsystem {
    public static final double FLAP_CLOSED_POSITION = 0.65;
    public static final double FLAP_OPENED_POSITION = 0;
    public static final double SPINNER_OFF_POWER = 0;
    public static final double SPINNER_COLLECTING_POWER = -0.3;
    public static final double SPINNER_DEPOSITING_POWER = 0.09;

    public static final long FLAP_DELAY = 250;
    public static final long SPINNER_DELAY = 250;
    public static final long PIXEL_DEPOSIT_TIME = 1750;

    public static final double MIN_ANGLE = 0;
    public static final double MAX_ANGLE = 1;

    private final CRServo spinner;
    private final ServoEx flap;
    private final Location location;

    public PixelCollector(Location location, HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        this.location = location;
        String name = location == Location.LEFT ? "collectorLeft" : "collectorRight";

        spinner = new CRServo(hardwareMap, name + "Spinner");
        flap = new SimpleServo(hardwareMap, name + "Flap", MIN_ANGLE, MAX_ANGLE);
        if (location == Location.LEFT) {
            flap.setInverted(true);
            spinner.setInverted(true);
        }

        telemetry.addLine("[PC " + location + "] initialized");
    }

    /**
     * Sets the position of the pixel collector flap.
     *
     * @param position the position of the pixel collector flap.
     */
    public void setFlapPosition(double position) {
        flap.setPosition(position);
        telemetry.addData("[PC " + location + "] flap position", position);
    }

    /**
     * Gets the location of the pixel collector.
     *
     * @return the location of the pixel collector.
     */
    public Location getLocation() {
        return location;
    }

    /**
     * Get the current position of the flap.
     *
     * @return the current position of the flap
     */
    public double getFlapPosition() {
        return flap.getPosition();
    }

    /**
     * Get the current power of the spinner.
     *
     * @return the current power of the spinner
     */
    public double getSpinnerPower() {
        return spinner.get();
    }

    /**
     * Sets the power for the pixel collector spinner.
     *
     * @param power the power for the pixel collector spinner.
     */
    public void setSpinnerPower(double power) {
        spinner.set(power);
        telemetry.addData("[PC " + location + "] spinner power", power);
    }

    /**
     * Location of the pixel collector
     */
    public enum Location {
        LEFT,
        RIGHT
    }

    /**
     * Gets a command to deposit a pixel.
     *
     * @return a command to deposit a pixel.
     */
    public CommandEx depositPixel() {
        return new DepositPixel(this);
    }

    /**
     * Command to deposit a pixel. This manages the flap doors as well as the spinner.
     */
    private static class DepositPixel extends CommandBaseEx {
        private final PixelCollector pixelCollector;
        private final ElapsedTime timer = new ElapsedTime();
        private State state;

        /**
         * Instantiate a new command to deposit a pixel.
         * @param pixelCollector the pixel collector from which to deposit a pixel.
         */
        public DepositPixel(PixelCollector pixelCollector) {
            this.pixelCollector = pixelCollector;
            addRequirements(pixelCollector);
        }

        /**
         * Initialize the command. This sets the flap position to the opened position and starts
         * a timer to wait for the change to be realized.
         */
        @Override
        public void initialize() {
            pixelCollector.setFlapPosition(FLAP_OPENED_POSITION);
            state = State.FLAP_OPENING;
            timer.reset();
        }

        /**
         * Deposits a pixel. This:
         * <ul>
         *     <li>
         *         opens the flap door (done in the initialize method), waits for
         *          it to be opened
         *     </li>
         *     <li>
         *         starts the spinner
         *     </li>
         *     <li>
         *         waits for the pixel to be deposited
         *     </li>
         *     <li>
         *         stops the spinner, and waits for it to stop
         *     </li>
         *     <li>
         *         closes the flap door
         *     </li>
         * </ul>
         */
        @Override
        public void execute() {
            switch (state) {
                // Wait for the flap door to open, then start the spinner
                case FLAP_OPENING:
                    if (timer.milliseconds() > FLAP_DELAY) {
                        pixelCollector.setSpinnerPower(SPINNER_DEPOSITING_POWER);
                        state = State.SPINNER_RUNNING;
                        timer.reset();
                    }
                    break;
                // Wait for the pixel to be deposited and then stop the spinner
                case SPINNER_RUNNING:
                    if (timer.milliseconds() > PIXEL_DEPOSIT_TIME) {
                        pixelCollector.setSpinnerPower(SPINNER_OFF_POWER);
                        state = State.SPINNER_STOPPING;
                        timer.reset();
                    }
                    break;
                // Wait for the spinner to stop,and then close the flap door
                case SPINNER_STOPPING:
                    if (timer.milliseconds() > SPINNER_DELAY) {
                        pixelCollector.setFlapPosition(FLAP_CLOSED_POSITION);
                        state = State.FLAP_CLOSING;
                        timer.reset();
                    }
                    break;
                // Move to the stopped state. Right now, there is not timer to wait for this to
                // complete, but one could be added if needed
                case FLAP_CLOSING:
                    state = State.FLAP_CLOSED;
                    break;
            }
        }

        /**
         * Returns an indication as to whether the pixel has been deposited and the pixel
         * collector has returned to the initial "off" position of the pixel door being closed
         * and the spinner stopped.
         *
         * @return <code>true</code> if the command has finished depositing the pixel;
         * <code>false</code> if the command is still in the process of depositing the pixel.
         */
        @Override
        public boolean isFinished() {
            return state != null && state == State.FLAP_CLOSED;
        }

        /**
         * Internal state for the command to deposit a pixel
         */
        private enum State {
            FLAP_OPENING,
            SPINNER_RUNNING,
            FLAP_CLOSING,
            SPINNER_STOPPING,
            FLAP_CLOSED
        }
    }
}
