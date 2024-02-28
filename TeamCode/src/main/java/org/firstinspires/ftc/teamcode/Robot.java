package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.DroneLauncher;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PixelCollector;

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

    /**
     * Creates a new instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     */
    private Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        robot = this;
        this.telemetry = telemetry;

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        droneLauncher = new DroneLauncher(hardwareMap, telemetry);
        leftPixelCollector = new PixelCollector(hardwareMap, telemetry, "collectorLeft", true);
        rightPixelCollector = new PixelCollector(hardwareMap, telemetry, "collectorRight", false);

        this.telemetry.addLine("[Robot] initialized");
    }

    /**
     * Gets the singleton instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     */
    public static Robot getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (robot == null) {
            robot = new Robot(hardwareMap, telemetry);
        }
        return robot;
    }
}
