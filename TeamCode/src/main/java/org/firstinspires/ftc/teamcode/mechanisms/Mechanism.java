package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Mechanism is the base class for all mechanisms on the robot.
 */
public interface Mechanism {
    /**
     * getDeviceName returns the name of the device (e.g., "leftFront" for a motor).
     * @return the name of the device.
     */
    public abstract String getDeviceName();

    /**
     * getDescription returns a description of the device (e.g., "Left Front Motor").
     * @return a description of the device.
     */
    public abstract String getDescription();

    /**
     * getTests returns the list of tests for the Mechanism.
     *
     * @return the list of tests for the Mechanism.
     */
    public abstract Collection<Test> getTests();

    /**
     * string returns a string representation for the Mechanism. By default,
     * this is the simple name for the Mechanism class, along with the device
     * name and description.
     *
     * @return a string representation for the mechanism.
     */
    default String string() {
        StringBuilder str = new StringBuilder();
        str.append("DeviceType: ");
        str.append(this.getClass().getSimpleName());
        str.append(", DeviceName: ");
        str.append(getDescription());
        str.append(", Description: ");
        str.append(getDescription());
        return str.toString();
    }
}