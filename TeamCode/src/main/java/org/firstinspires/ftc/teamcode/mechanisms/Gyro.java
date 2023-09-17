package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

public class Gyro implements Mechanism {

    private IMU gyro;
    private final String deviceName;
    private final String description;

    public Gyro(Robot robot, String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }
    @Override
    public void init(HardwareMap hwMap) {
        gyro = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU with this mounting orientation
        gyro.initialize(new IMU.Parameters(orientationOnRobot));
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
