package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Mechanism is the base class for all mechanisms on the robot.
 */
abstract public class Mechanism {
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
        return this.getClass().getSimpleName();
    }
}