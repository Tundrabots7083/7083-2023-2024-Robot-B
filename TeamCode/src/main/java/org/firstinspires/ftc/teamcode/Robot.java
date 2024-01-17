package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controls.ArmController;
import org.firstinspires.ftc.teamcode.controls.Controller;
import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public static Robot robot;
    public MecanumDriveController mecanumDriveController;
    public PixelMoverController pixelMoverController;
    public ArmController armController;
    private final Telemetry telemetry;
    public final List<Controller> controllers;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        robot = this;
        this.telemetry = telemetry;

        mecanumDriveController = new MecanumDriveController(hardwareMap, telemetry);
        pixelMoverController = new PixelMoverController(hardwareMap, telemetry);
        armController = new ArmController(hardwareMap, telemetry);
        controllers = Arrays.asList(mecanumDriveController, pixelMoverController, armController);

        telemetry.addLine("[Robot] controllers initialized");
    }

    public static Robot getRobot() {
        return robot;
    }
}
