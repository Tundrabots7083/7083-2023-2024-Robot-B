package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;

import java.util.Arrays;
import java.util.List;

/**
 * Robot represents all mechanisms and hardware that resides on the robot.
 */
public class Robot {
    public final DriveTrain drive = new DriveTrain(this, "driveTrain", "Drive Train");
    public final Gyro imu = new Gyro(this,"imu", "IMU");
    private final List<Mechanism> mechanisms = Arrays.asList(drive, imu);

    public Robot() {
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
