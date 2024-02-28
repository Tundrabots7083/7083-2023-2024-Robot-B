package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for controlling a single pixel collector.
 * A pixel collector is defined as a flap servo + a spinner servo.
 * Robot B has two pixel collectors
 */
@Config
public class PixelCollector implements Mechanism {
    /**
     * The number of milliseconds to delay between changing the flap servo and changing the spinner servo
     */
    public static long SPINNER_DELAY = 250;
    private final String deviceName;
    private final String description;
    private final Telemetry telemetry;
    private final CRServo spinner;
    private final Servo flap;
    private long spinnerDelayTime;
    private State state;

    /**
     * Creates an instance of a pixel collector.
     *
     * @param hardwareMap      the mapping of all hardware on the bot.
     * @param telemetry        the telemetry used to display data on the driver station.
     * @param hardwarePrefix   the prefix for the spinner and flap servos.
     * @param reverseFlapServo whether the servo runs forward or reverse.
     */
    public PixelCollector(HardwareMap hardwareMap, Telemetry telemetry, String hardwarePrefix, boolean reverseFlapServo) {
        this.deviceName = hardwarePrefix;
        this.description = hardwarePrefix + " pixel collector";

        this.spinner = hardwareMap.get(CRServo.class, hardwarePrefix + "Spinner");
        this.flap = hardwareMap.get(Servo.class, hardwarePrefix + "Flap");
        this.telemetry = telemetry;

        if (reverseFlapServo) {
            this.flap.setDirection(Servo.Direction.REVERSE);
            this.spinner.setDirection(CRServo.Direction.REVERSE);
        }

        this.state = State.CLOSED;
        this.flap.setPosition(this.state.flapPosition);
        this.spinner.setPower(this.state.spinnerPower);

        spinnerDelayTime = System.currentTimeMillis();
    }

    /**
     * Gets indication as to whether the pixel collector is in a stopped state.
     *
     * @return <code>true</code> if the pixel collector is in a stopped state.
     * <code>false</code> otherwise.
     */
    public boolean isStopped() {
        return (flap.getPosition() == State.CLOSED.flapPosition && spinner.getPower() == State.CLOSED.spinnerPower);
    }

    /**
     * Sets the state for the pixel collector.
     *
     * @param state the state for the pixel collector.
     */
    public void setState(State state) {
        this.state = state;
        spinnerDelayTime = System.currentTimeMillis() + SPINNER_DELAY;
    }

    public void setSpinnerPower(double power) {
        spinner.setPower(power);
    }

    public void setFlapPosition(double position) {
        flap.setPosition(position);
    }

    /**
     * Update the position of the flap and the power for the spinner for the pixel collector.
     */
    public void update() {
        long currentTime = System.currentTimeMillis();
        if (state == State.CLOSED) {
            setSpinnerPower(state.spinnerPower);
            if (currentTime > spinnerDelayTime) {
                setFlapPosition(state.flapPosition);
            }
        } else {
            // Set the flap servo position to the current target
            setFlapPosition(state.flapPosition);

            // Check if we can set the spinner position yet
            if (currentTime > spinnerDelayTime) {
                setSpinnerPower(state.spinnerPower);
            }
        }
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    @Override
    public String getDescription() {
        return description;
    }

    /**
     * The state for the pixel collector.
     */
    public enum State {
        /**
         * Collecting pixels from the field
         */
        COLLECTING(-0.3, 0),
        /**
         * Depositing pixels on the field
         */
        DEPOSITING(0.09, 0),
        /**
         * The spinner is off and the flap door is closed
         */
        CLOSED(0, 0.65);

        public final double spinnerPower;
        public final double flapPosition;

        State(double spinnerPower, double flapPosition) {
            this.spinnerPower = spinnerPower;
            this.flapPosition = flapPosition;
        }
    }
}
