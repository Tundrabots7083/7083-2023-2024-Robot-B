package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import java.util.Arrays;
import java.util.Collection;

/**
 * Robot represents all mechanisms and hardware that resides on the robot.
 */
public class Robot {

    private final Collection<Mechanism> mechanisms = Arrays.asList();

    public void init(HardwareMap hwMap) {
    }

    /**
     * getMechanisms returns a collection of all mechanisms on the robot.
     *
     * @return a collection of all mechanisms on the robot.
     */
    public Collection<Mechanism> getMechanisms() {
        return mechanisms;
    }
}
