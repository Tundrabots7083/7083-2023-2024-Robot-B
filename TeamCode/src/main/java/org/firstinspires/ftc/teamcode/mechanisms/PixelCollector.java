package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Class for controlling a single pixel collector.
 * A pixel collector is defined as a flap servo + a spinner servo.
 * Robot B has two pixel collectors
 */
public class PixelCollector implements Mechanism {

    public static final long TOGGLE_DELAY = 500;

    String deviceName;
    String description;
    Telemetry telemetry;
    ServoEx spinner;
    ServoEx flap;
    long lastToggleTime;
    PixelCollectorState state;

    enum PixelCollectorState {
        COLLECTING(1, 0),
        DEPOSITING(0, 0),
        CLOSED(0.5, 1);

        double spinnerPosition;
        double flapPosition;

        PixelCollectorState(double spinnerPosition, double flapPosition) {
            this.spinnerPosition = spinnerPosition;
            this.flapPosition = flapPosition;
        }


    }

    public PixelCollector(String deviceName, String description, HardwareMap hardwareMap, Telemetry telemetry) {

        this.spinner = hardwareMap.get(ServoEx.class, deviceName + "_spinner");
        this.flap = hardwareMap.get(ServoEx.class, deviceName + "_flap");
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.description = description;

        state = PixelCollectorState.CLOSED;
    }

    public void toggleState(boolean deposit) {
        if (lastToggleTime + TOGGLE_DELAY < System.currentTimeMillis()) {
            if (state != PixelCollectorState.CLOSED) {
                state = PixelCollectorState.CLOSED;
            } else if (deposit) {
                state = PixelCollectorState.DEPOSITING;
            }
            else {
                state = PixelCollectorState.COLLECTING;
            }

            lastToggleTime = System.currentTimeMillis();

            spinner.setPosition(state.spinnerPosition);
            flap.setPosition(state.flapPosition);
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

    @Override
    public Collection<Test> getTests() {
        return null;
    }


}
