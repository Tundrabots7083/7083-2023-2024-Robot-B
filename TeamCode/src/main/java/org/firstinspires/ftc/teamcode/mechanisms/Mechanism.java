package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Mechanism is the base class for all mechanisms on the robot.
 */
abstract public class Mechanism {

    public final String deviceName;
    public final String description;

    /**
     * Creates a new mechanism.
     * @param deviceName the name of the mechanism.
     * @param description a description of the mechanism.
     */
    public Mechanism(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    /**
     * init initializes the Mechanism.
     * 
     * @param hwMap the map of all hardware on the robot.
     */
    public abstract void init(HardwareMap hwMap);

    /**
     * getTests returns the list of tests for the Mechanism.
     *
     * @return the list of tests for the Mechanism.
     */
    public abstract Collection<Test> getTests();

    /**
     * toString returns a string representation for the Mechanism. By default,
     * this is the simple name for the Mechanism class.
     *
     * @return the string representation for the mechanism.
     */
    public String toString() {
        StringBuilder str = new StringBuilder();
        str.append("DeviceType: ");
        str.append(this.getClass().getSimpleName());
        str.append(", DeviceName: ");
        str.append(deviceName);
        str.append(", Description: ");
        str.append(description);
        return str.toString();
    }
}