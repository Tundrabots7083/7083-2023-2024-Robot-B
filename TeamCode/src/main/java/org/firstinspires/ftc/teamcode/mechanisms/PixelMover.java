package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Implements the intake mechanism for the Robot.
 */
public class PixelMover implements Mechanism {
    private final String deviceName;
    private final String description;
    private CRServo brushRoller;
    private CRServo containerRoller;
    private Servo containerBackPull;
    private static final double STOPPED_POSITION = 0.5d;
    private static final double PULL_POSITION = 0.0d;
    private static final double STOPPED_POWER = 0.0d;
    private static final double FORWARD_POWER = 0.5d;
    private static final double REVERSE_POWER = -0.5d;
    private static enum PixelMoverState {
        FORWARD,
        REVERSE,
        STOPPED
    }
    private PixelMoverState state = PixelMoverState.STOPPED;

    public PixelMover(Robot robot, String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    @Override
    public void init(HardwareMap hwMap) {
        brushRoller = hwMap.get(CRServo.class, "brushRoller");
        initServo(brushRoller);
        containerRoller = hwMap.get(CRServo.class, "containerRoller");
        initServo(containerRoller);
        containerBackPull = hwMap.get(Servo.class, "containerBackPull");
        initServo(containerBackPull);
    }

    private void initServo(CRServo servo) {
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPower(STOPPED_POWER);
    }

    private void initServo(Servo servo) {
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(STOPPED_POSITION);
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

    /**
     * Toggles mechanism between being stopped and moving forward.
     *
     */
    public void toggleForward() {
        if (PixelMoverState.STOPPED.equals(state)) {
            // Pixel mover is stopped, get it moving forward
            brushRoller.setPower(FORWARD_POWER);
            containerRoller.setPower(1.5 * REVERSE_POWER);
            containerBackPull.setPosition(PULL_POSITION);
            state = PixelMoverState.FORWARD;
        }
        else if (PixelMoverState.FORWARD.equals(state)) {
            // Pixel mover is rotating forward (clockwise), stop it.
            brushRoller.setPower(STOPPED_POWER);
            containerRoller.setPower(STOPPED_POWER);
            containerBackPull.setPosition(PULL_POSITION);
            state = PixelMoverState.STOPPED;
        }
    }

    /**
     * Toggles mechanism between being stopped and moving in reverse.
     *
     */
    public void toggleReversed() {
        if (PixelMoverState.STOPPED.equals(state)) {
            // Pixel mover is stopped, get it moving forward
            brushRoller.setPower(REVERSE_POWER);
            containerRoller.setPower(1.5 * FORWARD_POWER);
            containerBackPull.setPosition(STOPPED_POSITION);
            state = PixelMoverState.REVERSE;
        }
        else if (PixelMoverState.REVERSE.equals(state)) {
            // Pixel mover is rotating forward (clockwise), stop it.
            brushRoller.setPower(STOPPED_POWER);
            containerRoller.setPower(STOPPED_POWER);
            containerBackPull.setPosition(STOPPED_POSITION);
            state = PixelMoverState.STOPPED;
        }
    }

    @Override
    public String toString() {
        return string();
    }
}
