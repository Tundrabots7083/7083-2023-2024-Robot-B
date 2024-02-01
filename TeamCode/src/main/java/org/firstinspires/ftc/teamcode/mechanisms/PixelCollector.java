package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    CRServo spinner;
    Servo flap;
    long lastToggleTime;
    PixelCollectorState state;
    boolean leftServo;

    enum PixelCollectorState {
        COLLECTING(0.3, 1),
        DEPOSITING(-0.2, 1),
        CLOSED(0, 0);

        double spinnerPower;
        double flapPosition;

        PixelCollectorState(double spinnerPower, double flapPosition) {
            this.spinnerPower = spinnerPower;
            this.flapPosition = flapPosition;
        }


    }

    public PixelCollector(String deviceName, String description, HardwareMap hardwareMap, Telemetry telemetry, boolean leftServo) {

        this.spinner = hardwareMap.get(CRServo.class, deviceName + "Spinner");
        this.flap = hardwareMap.get(Servo.class, deviceName + "Flap");
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

            spinner.setPower(state.spinnerPower);
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
