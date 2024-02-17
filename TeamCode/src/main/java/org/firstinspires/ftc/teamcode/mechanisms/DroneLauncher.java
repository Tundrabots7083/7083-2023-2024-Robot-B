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

    public static double SERVO_LAUNCH_START_ANGLE = 0.125;
    public static double SERVO_LAUNCH_ANGLE_HIGH = 0.36; // Using tension of 4.25, and bot centered on outer spike mark
    public static double SERVO_LAUNCH_ANGLE_LOW = 0.36; // Use when there is a noticible airflow
    public static boolean SERVO_LAUNCH_ANGLE_USE_HIGH = true;
    public static double SERVO_RELEASE_INITIAL = 0.675;
    public static double SERVO_RELEASE_POS = 0.2;
    private final String deviceName;
    private final String description;
    private Servo releaseServo;
    private Servo angleServo;

    /**
     * Creates the drone launcher
     * @param deviceName name of the drone launcher device.
     * @param description description of the drone launcher.
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     */
    public DroneLauncher(String deviceName, String description, HardwareMap hardwareMap) {
        this.deviceName = deviceName;
        this.description = description;

        releaseServo = hardwareMap.get(Servo.class, "droneLauncher");
        releaseServo.setDirection(Servo.Direction.FORWARD);
        releaseServo.setPosition(SERVO_RELEASE_INITIAL); // TODO: verify drone launch servo doesn't move

        angleServo = hardwareMap.get(Servo.class, "dronePosition");
        angleServo.setDirection(Servo.Direction.REVERSE);
        angleServo.setPosition(SERVO_LAUNCH_START_ANGLE); // TODO: verify angle is flat
    }

    /**
     * Moves the drone launcher into the down position
     */
    public void setToStartAngle() {
        angleServo.setPosition(SERVO_LAUNCH_START_ANGLE);
    }

    /**
     * Moves the drone launcher into launch position
     */
    public void setToLaunchAngle() {
        double servoAnglePosition = SERVO_LAUNCH_ANGLE_USE_HIGH ? SERVO_LAUNCH_ANGLE_HIGH : SERVO_LAUNCH_ANGLE_LOW;
        angleServo.setPosition(servoAnglePosition);
    }

    /**
     * Launches the drone, hopefully scoring a lot of points in the process.
     */
    public void launchDrone() {
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