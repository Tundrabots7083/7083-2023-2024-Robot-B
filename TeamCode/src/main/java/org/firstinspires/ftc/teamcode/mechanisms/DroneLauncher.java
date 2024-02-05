package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * DroneLauncher manages the launching of the drone (paper airplane)
 */
@Config
public class DroneLauncher implements Mechanism {
    public static double LAUNCH_POS = 0.5;
    public static double SERVO_RELEASE_POS = 0.1;
    private final String deviceName;
    private final String description;
    private Servo releaseServo;
    private Servo positionServo;

    /**
     * Createss the drone launcher
     * @param deviceName
     * @param description
     * @param hardwareMap
     */
    public DroneLauncher(String deviceName, String description, HardwareMap hardwareMap) {
        this.deviceName = deviceName;
        this.description = description;

        releaseServo = hardwareMap.get(Servo.class, deviceName);
        releaseServo.setDirection(Servo.Direction.FORWARD);

        positionServo = hardwareMap.get(Servo.class, "dronePosition");
        positionServo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Moves the drone launcher into launch position
     */
    public void setToLaunchPosition() {
        positionServo.setPosition(LAUNCH_POS);
    }

    /**
     * Launches the drone, hopefully scoring a lot of points in the process.
     */
    public void launch() {
        releaseServo.setPosition(SERVO_RELEASE_POS);
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

    @Override
    @NonNull
    public String toString() {
        return string();
    }
}