package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
}
