package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class IMU implements Mechanism {

    private com.qualcomm.robotcore.hardware.IMU gyro;
    private final String deviceName;
    private final String description;

    public IMU(Robot robot, String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }
    @Override
    public void init(HardwareMap hwMap) {
        gyro = hwMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
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
     * getHeading gets the direction the robot is facing.
     * @param angleUnit the unit of measure for the angle.
     * @return the direction the robot is facing.
     */
    public double getHeading(AngleUnit angleUnit) {
        YawPitchRollAngles angles = getRobotYawPitchRollAngles();
        return angles.getYaw(angleUnit);
    }

    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return gyro.getRobotYawPitchRollAngles();
    }

    @Override
    public String toString() {
        return string();
    }
}
