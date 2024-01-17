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
    public static double SERVO_RELEASE_POS = 0;
    private final String deviceName;
    private final String description;
    private Servo servo;

    public DroneLauncher(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    /**
     * Initializes the drone launcher.
     * @param hardwareMap the map of all hardware on the robot.
     */
    @Override
    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, deviceName);
    }

    /**
     * Launches the drone, hopefully scoring a lot of points in the process.
     */
    public void launch() {
        servo.setPosition(SERVO_RELEASE_POS);
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