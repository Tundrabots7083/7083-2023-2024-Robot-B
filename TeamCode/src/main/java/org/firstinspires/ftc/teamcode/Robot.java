package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanism.DroneLauncher;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.PixelCollector;
import org.firstinspires.ftc.teamcode.sensor.VisionSensor;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
public class Robot {
    private static Robot robot = null;

    public final Telemetry telemetry;

    // Mechanisms
    public final MecanumDrive mecanumDrive;
    public final DroneLauncher droneLauncher;
    public final Lift lift;
    public final PixelCollector leftPixelCollector, rightPixelCollector;
    public final VisionSensor visionSensor;

    /**
     * Creates a new instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param createVisionSensor <code>true</code> if the vision sensor is to be created;
     *                           <code>false</code> otherwise.
     */
    private Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean createVisionSensor) {
        robot = this;
        this.telemetry = telemetry;

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        droneLauncher = new DroneLauncher(hardwareMap, telemetry);

        leftPixelCollector = new PixelCollector("collectorLeft", "Left pixel collector", hardwareMap, telemetry, true);
        rightPixelCollector = new PixelCollector("collectorRight", "Right pixel collector", hardwareMap, telemetry, false);

        if (createVisionSensor) {
            // Create the vision sensor
            visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"), telemetry);
        } else {
            visionSensor = null;
        }

        this.telemetry.addLine("[Robot] initialized");
    }

    /**
     * Gets the singleton instance of the robot. The vision sensor is not created when calling
     * this method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     */
    public static Robot getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        return getInstance(hardwareMap, telemetry, false);
    }

    /**
     * Gets the singleton instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param createVisionSensor <code>true</code> if the vision sensor is to be created;
     *                           <code>false</code> otherwise.
     */
    public static Robot getInstance(HardwareMap hardwareMap, Telemetry telemetry, boolean createVisionSensor) {
        if (robot == null) {
            robot = new Robot(hardwareMap, telemetry, createVisionSensor);
        }
        return robot;
    }
}
