package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class Claw implements Mechanism {

    private static final double GRAB_POS = 0.54;
    private static final double RELEASE_POS = 0.35;
    private static final double OPEN_POS = 0.75;
    private Servo claw;
    private final String deviceName;
    private final String description;

    public Claw(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "deviceName");
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

    public void grab()
    {
        claw.setPosition(GRAB_POS);
    }

    public void release()
    {
        claw.setPosition(RELEASE_POS);
    }

    public void open()
    {
        claw.setPosition(OPEN_POS);
    }

    @Override
    public String toString() {
        return string();
    }

}