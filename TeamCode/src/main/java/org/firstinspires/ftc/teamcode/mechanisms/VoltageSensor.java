package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class VoltageSensor implements Mechanism {
    private com.qualcomm.robotcore.hardware.VoltageSensor voltageSensor;
    private final String deviceName;
    private final String description;

    public VoltageSensor(Robot robot, String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    @Override
    public void init(HardwareMap hwMap) {
        voltageSensor = hwMap.voltageSensor.get(deviceName);
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getDescription() {
        return null;
    }

    @Override
    public Collection<Test> getTests() {
        return null;
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String toString() {
        return string();
    }
}
