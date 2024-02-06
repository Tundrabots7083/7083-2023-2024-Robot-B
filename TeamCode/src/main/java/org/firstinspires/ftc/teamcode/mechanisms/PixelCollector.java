package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class PixelCollector implements Mechanism {


    public static long TOGGLE_DELAY = 500;

    /**
     * The number of milliseconds to delay between changing the flap servo and changing the spinner servo
     */
    public static long SPINNER_DELAY = 250;

    String deviceName;
    String description;
    Telemetry telemetry;
    CRServo spinner;
    Servo flap;
    long lastToggleTime;
    long spinnerDelayTime;
    PixelCollectorState state;
    boolean leftServo;

    enum PixelCollectorState {
        COLLECTING(-0.3, 0),
        DEPOSITING(0.1, 0),
        CLOSED(0, 0.65);

        double spinnerPower;
        double flapPosition;

        PixelCollectorState(double spinnerPower, double flapPosition) {
            this.spinnerPower = spinnerPower;
            this.flapPosition = flapPosition;
        }


    }

    public PixelCollector(String deviceName, String description, HardwareMap hardwareMap, Telemetry telemetry, boolean reverseFlapServo) {

        this.spinner = hardwareMap.get(CRServo.class, deviceName + "Spinner");
        this.flap = hardwareMap.get(Servo.class, deviceName + "Flap");
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.description = description;

        if (reverseFlapServo) {
            this.flap.setDirection(Servo.Direction.REVERSE);
            this.spinner.setDirection(CRServo.Direction.REVERSE);
        }

        state = PixelCollectorState.CLOSED;

        spinnerDelayTime = System.currentTimeMillis();
    }

    public void toggleState(boolean deposit) {
        if (lastToggleTime + TOGGLE_DELAY < System.currentTimeMillis()) {
            if (state != PixelCollectorState.CLOSED) {
                state = PixelCollectorState.CLOSED;
//                spinner.setPower(state.spinnerPower);
//                flap.setPosition(state.flapPosition);
            } else if (deposit) {
                state = PixelCollectorState.DEPOSITING;
//                spinner.setPower(state.spinnerPower);
//                flap.setPosition(state.flapPosition);
            }
            else {
                state = PixelCollectorState.COLLECTING;
                spinnerDelayTime = System.currentTimeMillis() + SPINNER_DELAY;
//                flap.setPosition(state.flapPosition);
//                spinner.setPower(state.spinnerPower);
            }

            lastToggleTime = System.currentTimeMillis();
        }
    }

    public void update() {

        // Set the flap servo position to the current target
        flap.setPosition(state.flapPosition);

        // Check if we can set the spinner position yet
        long currentTime = System.currentTimeMillis();

        if (currentTime > spinnerDelayTime) {
            spinner.setPower(state.spinnerPower);
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
