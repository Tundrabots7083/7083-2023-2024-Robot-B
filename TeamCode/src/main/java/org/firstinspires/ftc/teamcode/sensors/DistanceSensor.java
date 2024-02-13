package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Obtains the distance from the sensor to the object in front of the sensor.
 */
public class DistanceSensor {
    private final Telemetry telemetry;
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    /**
     * Creates a DistanceSensor object from the included hardware map. The distance sensor must be
     * named `distanceSensor`.
     * @param hardwareMap the hardware map for the robot.
     * @param telemetry the telemetry to be used for any output to the driver station.
     * @param deviceName the name of the distance sensor, as configured on the robot.
     */
    public DistanceSensor(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
        this.telemetry = telemetry;
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, deviceName);
    }

    /**
     * Returns the current distance in inches
     * @return the current distance sas measured by the sensor. If no reading is available
     *         (perhaps the sensor is out of range), then distanceOutOfRange is returned;
     */
    public double getDistance() {
        return getDistance(DistanceUnit.INCH);
    }

    /**
     * Returns the current distance in the indicated distance units
     * @param du the unit of distance in which the result should be returned
     * @return the current distance sas measured by the sensor. If no reading is available
     *         (perhaps the sensor is out of range), then distanceOutOfRange is returned;
     */
    public double getDistance(DistanceUnit du) {
        return distanceSensor.getDistance(du);
    }
}