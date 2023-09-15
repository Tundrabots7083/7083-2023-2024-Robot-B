package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.DriveChassis;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import java.util.Arrays;
import java.util.List;

/**
 * Robot represents all mechanisms and hardware that resides on the robot.
 */
public class Robot {
    public final DriveChassis drive;
    private final List<Mechanism> mechanisms;

    public Robot() {
        this.drive = new DriveChassis("driveChassis", "Drive Chassis");
        this.mechanisms = Arrays.asList(drive);
    }

    /**
     * Initialize initializes all the hardware on the Robot. This should be called once when the
     * `Init` button is pressed.
     *
     * @param hwMap the map of all hardware on the Robot.
     */
    public void init(HardwareMap hwMap) {
        for (Mechanism mechanism: mechanisms) {
            mechanism.init(hwMap);
        }
    }

    /**
     * getMechanisms returns a collection of all mechanisms on the robot.
     *
     * @return a collection of all mechanisms on the robot.
     */
    public List<Mechanism> getMechanisms() {
        return mechanisms;
    }
}
