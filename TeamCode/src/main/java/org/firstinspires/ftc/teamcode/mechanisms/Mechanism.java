package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Mechanism is the base class for all mechanisms on the robot.
 */
public interface Mechanism {
    /**
     * getDeviceName returns the name of the device (e.g., "leftFront" for a motor).
     *
     * @return the name of the device.
     */
    String getDeviceName();

    /**
     * getDescription returns a description of the device (e.g., "Left Front Motor").
     *
     * @return a description of the device.
     */
    String getDescription();

    /**
     * getTests returns the list of tests for the Mechanism.
     *
     * @return the list of tests for the Mechanism.
     */
    Collection<Test> getTests();

    /**
     * string returns a string representation for the Mechanism. By default,
     * this is the simple name for the Mechanism class, along with the device
     * name and description.
     *
     * @return a string representation for the mechanism.
     */
    default String string() {
        String str = "DeviceType: " +
                this.getClass().getSimpleName() +
                ", DeviceName: " +
                getDescription() +
                ", Description: " +
                getDescription();
        return str;
    }
}