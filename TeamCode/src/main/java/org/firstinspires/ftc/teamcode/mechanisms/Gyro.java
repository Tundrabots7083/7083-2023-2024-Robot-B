package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class Gyro implements Mechanism {

    private com.qualcomm.robotcore.hardware.IMU gyro;
    private final String deviceName;
    private final String description;

    public Gyro(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }
    @Override
    public void init(HardwareMap hwMap) {
        gyro = hwMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with this mounting orientation
        gyro.initialize(new com.qualcomm.robotcore.hardware.IMU.Parameters(orientationOnRobot));
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
     * getYaw returns the side-to-side lateral rotation of the robot (rotation around the Z axis),
     * normalized to the range of [-180,+180) degrees in Radians.
     *
     * @return the side-to-side lateral rotation of the robot (rotation around the Z axis),
     *         normalized to the range of [-180,+180) degrees.
     */
    public double getYaw() {
        return getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * getRobotYawPitchRollAngles returns a YawPitchRollAngles object representing the current
     * orientation of the robot relative to the robot's position the last time that resetYaw()
     * was called, as if the robot was perfectly level at that time.
     *
     * @return a YawPitchRollAngles object representing the current orientation of the robot
     *         relative to the robot's position the last time that resetYaw() was called, as if
     *         the robot was perfectly level at that time.
     */
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return gyro.getRobotYawPitchRollAngles();
    }

    /**
     * Resets the robot's yaw angle to 0. After calling this method, the reported orientation will
     * be relative to the robot's position when this method was called, as if the robot was
     * perfectly level right then. That is to say, the pitch and yaw will be ignored when this
     * method is called. Unlike yaw, pitch and roll are always relative to gravity, and never
     * need to be reset.
     */
    public void resetYaw() {
        gyro.resetYaw();
    }

    @Override
    public String toString() {
        return string();
    }
}
