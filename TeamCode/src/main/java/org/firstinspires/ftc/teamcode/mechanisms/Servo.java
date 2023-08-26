package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class Servo extends Mechanism {

    private com.qualcomm.robotcore.hardware.Servo servo;

    public Servo(String deviceName, String description) {
        super(deviceName, description);
    }

    @Override
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(com.qualcomm.robotcore.hardware.Servo.class, deviceName);
    }

    @Override
    public Collection<Test> getTests() {
        return null;
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }
}
