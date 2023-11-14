package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controls.Controller;
import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public final MecanumDriveController mecanumDriveController = new MecanumDriveController();
    public final List<Controller> controllers = Arrays.asList(mecanumDriveController);

    public Robot() {}

    public void init(HardwareMap hardwareMap) {
        for (Controller controller : controllers) {
            controller.init(hardwareMap);
        }
    }
}
