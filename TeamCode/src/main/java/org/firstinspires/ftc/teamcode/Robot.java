package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.Controller;
import org.firstinspires.ftc.teamcode.controllers.DroneLauncherController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controllers.PixelCollectorController;
import org.firstinspires.ftc.teamcode.sensors.Sensor;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
public class Robot {
    private static Robot robot = null;
    public final List<Controller> controllers;
    public final List<Sensor> sensors;
    private final Telemetry telemetry;
    public MecanumDriveController mecanumDriveController;
    public LiftController liftController;
    public PixelCollectorController pixelCollectorController;
    public DroneLauncherController droneLauncherController;

    /**
     * Creates a new instance of the robot.
     * @param hardwareMap         hardware map for the robot.
     * @param telemetry           telemetry class for displaying data.
     */
    private Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        robot = this;
        this.telemetry = telemetry;

        mecanumDriveController = new MecanumDriveController(hardwareMap, telemetry);
        liftController = new LiftController(hardwareMap, telemetry);
        pixelCollectorController = new PixelCollectorController(hardwareMap, telemetry);
        droneLauncherController = new DroneLauncherController(hardwareMap, telemetry);
        controllers = Arrays.asList(mecanumDriveController, liftController, pixelCollectorController, droneLauncherController);

        sensors = Collections.emptyList();

        this.telemetry.addLine("[Robot] initialized");
    }

    /**
     * Gets the singleton instance of the robot.
     * @param hardwareMap         hardware map for the robot.
     * @param telemetry           telemetry class for displaying data.
     */
    public static Robot getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (robot == null) {
            robot = new Robot(hardwareMap, telemetry);
        }
        return robot;
    }
}
