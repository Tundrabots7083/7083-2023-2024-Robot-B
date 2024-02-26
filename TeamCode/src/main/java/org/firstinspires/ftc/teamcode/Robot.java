package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.Controller;
import org.firstinspires.ftc.teamcode.controllers.DroneLauncherController;
import org.firstinspires.ftc.teamcode.controllers.LiftController;
import org.firstinspires.ftc.teamcode.controllers.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controllers.PixelCollectorController;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public static Robot robot;
    public final List<Controller> controllers;
    //    public final DistanceSensor leftDistanceSensor;
//    public final DistanceSensor rightDistanceSensor;
//    public final VisionSensor visionSensor;
//    public final List<Sensor> sensors;
    private final Telemetry telemetry;
    public MecanumDriveController mecanumDriveController;
    public LiftController liftController;
    public PixelCollectorController pixelCollectorController;
    public DroneLauncherController droneLauncherController;
    public RobotState state;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        robot = this;
        this.telemetry = telemetry;

        mecanumDriveController = new MecanumDriveController(hardwareMap, telemetry);
        liftController = new LiftController(hardwareMap, telemetry);
        pixelCollectorController = new PixelCollectorController(hardwareMap, telemetry);
        droneLauncherController = new DroneLauncherController(hardwareMap, telemetry);
        controllers = Arrays.asList(mecanumDriveController, liftController, pixelCollectorController, droneLauncherController);

//        leftDistanceSensor = new DistanceSensor(hardwareMap, telemetry, "leftDistanceSensor");
//        rightDistanceSensor = new DistanceSensor(hardwareMap, telemetry, "rightDistanceSensor");
//        sensors = Arrays.asList(leftDistanceSensor, rightDistanceSensor);
//        visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"), telemetry);
//        sensors = Arrays.asList(leftDistanceSensor, rightDistanceSensor, visionSensor);

        this.telemetry.addLine("[Robot] initialized");
    }

    public static Robot getRobot() {
        return robot;
    }
}
