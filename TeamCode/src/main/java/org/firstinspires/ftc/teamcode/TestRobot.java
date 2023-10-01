package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestRobot extends Robot {
    /**
     * Initialize initializes all the hardware on the Robot. This should be called once when the
     * `Init` button is pressed.
     *
     * @param hwMap the map of all hardware on the Robot.
     */
    public void init(HardwareMap hwMap) {
        pixelMover.init(hwMap);
    }
}
