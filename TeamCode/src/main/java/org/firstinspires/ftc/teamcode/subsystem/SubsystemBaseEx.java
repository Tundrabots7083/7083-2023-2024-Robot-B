package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

/**
 * Base subsystem used by our subsystems. This extends the FTCLib subsystem so that includes some
 * methods used by multiple subsystems.
 */
public abstract class SubsystemBaseEx extends SubsystemBase implements SubsystemEx {

    /**
     * Modifies the power value so that it is between the minimum and maximum values.
     * @param power
     * @param minPower
     * @return
     */
    protected double modifyMotorPower(double power, double minPower) {
        // Cap the motor power at 1 and -1
        power = Math.max(-1, Math.min(1, power));
        // If the power level of the motor is below the minimum threshold, set it to 0
        if (Math.abs(power) < minPower) {
            power = 0;
        }
        return power;
    }
}
