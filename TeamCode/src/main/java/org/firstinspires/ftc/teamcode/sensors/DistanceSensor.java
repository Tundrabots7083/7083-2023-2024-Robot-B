package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.feedback.KalmanFilter;

/**
 * Obtains the distance from the sensor to the object in front of the sensor.
 */
@Config
public class DistanceSensor implements Sensor {
    // Kalman filter initial values
    public static double INITIAL_STATE = 0.0;
    public static double INITIAL_ERROR_COVARIANCE = 1.0;
    public static double PROCESS_NOISE = 0.01;
    public static double MEASUREMENT_NOISE = 0.1;

    private final Telemetry telemetry;
    private final com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;
    private final KalmanFilter kalmanFilter;

    /**
     * Creates a DistanceSensor object from the included hardware map. The distance sensor must be
     * named `distanceSensor`.
     *
     * @param hardwareMap the hardware map for the robot.
     * @param telemetry   the telemetry to be used for any output to the driver station.
     * @param deviceName  the name of the distance sensor, as configured on the robot.
     */
    public DistanceSensor(HardwareMap hardwareMap, Telemetry telemetry, String deviceName) {
        this.telemetry = telemetry;
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, deviceName);
        kalmanFilter = new KalmanFilter(INITIAL_STATE, INITIAL_ERROR_COVARIANCE, PROCESS_NOISE, MEASUREMENT_NOISE);
    }

    /**
     * Reset the measured distances for the distance sensor. This may be used in cases such as
     * the robot's lift and arm being positioned for scoring on the backdrop.
     */
    public void reset() {
        kalmanFilter.reset();
    }

    /**
     * Updates the current distance
     */
    public void update() {
        // If the distance is available, update the kalman filter with the value
        double distance = getCurrentDistance();
        if (distance != com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange) {
            kalmanFilter.update(distance);
            telemetry.addData("[DistanceSensor] distance", distance);
            telemetry.addData("[DistanceSensor] state", kalmanFilter.getState());
        } else {
            telemetry.addLine("[DistanceSensor] Out-of-Range");
        }
    }

    /**
     * Gets the filtered distance in inches.
     *
     * @return the filtered distance as measured by the sensor. If no reading is available then
     * distanceOutofRange is returned.
     */
    public double getDistance() {
        double distance = kalmanFilter.getState();
        // If no distance has been measured, return distanceOutOfRange. Otherwise, returned the
        // measured distance.
        if (distance == 0) {
            return com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange;
        }
        return distance;
    }

    /**
     * Gets the current distance in inches.
     *
     * @return the distance as measured by the sensor. If no reading is available
     * (perhaps the sensor is out of range), then distanceOutOfRange is returned.
     */
    public double getCurrentDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}