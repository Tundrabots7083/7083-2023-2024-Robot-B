package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class Servo implements Mechanism {

    private com.qualcomm.robotcore.hardware.Servo servo;
    private final String deviceName;
    private final String description;

    public Servo(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    @Override
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(com.qualcomm.robotcore.hardware.Servo.class, deviceName);
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

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    @Override
    public String toString() {
        return string();
    }
}
