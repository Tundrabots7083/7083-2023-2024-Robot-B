package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class Claw extends Mechanism {

    private static final double GRAB_POS = 0.54;
    private static final double RELEASE_POS = 0.35;
    private static final double OPEN_POS = 0.75;
    private Servo claw;

    public Claw(String deviceName, String description) {
        super(deviceName, description);

        claw = new Servo(deviceName, description);
    }

    public void init(HardwareMap hwMap) {
        claw.init(hwMap);
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

}