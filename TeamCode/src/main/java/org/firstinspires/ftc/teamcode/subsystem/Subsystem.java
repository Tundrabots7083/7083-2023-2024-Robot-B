package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Base subsystem class.
 */
public abstract class Subsystem extends SubsystemBase {
    public final Telemetry telemetry;

    /**
     * Creates a new subsystem that uses the supplied telemetry for displaying output.
     *
     * @param telemetry the telemetry to use for output.
     */
    public Subsystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Make sure the power exceeds a minimum threshold value or is otherwise it set to zero.
     * @param power the power to check.
     * @param minPower the minimum power threshold.
     * @return the power, or zero if the power does not meet the minimum power thresholld.
     */
    protected static double modifyMotorPower(double power, double minPower) {
        // Cap the motor power at 1 and -1
        power = Math.max(-1, Math.min(1, power));
        // If the power level of the motor is below the minimum threshold, set it to 0
        if (Math.abs(power) < minPower) {
            power = 0;
        }
        return power;
    }
}
