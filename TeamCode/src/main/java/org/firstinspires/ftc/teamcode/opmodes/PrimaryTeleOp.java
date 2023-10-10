package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.Controller;
import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends OpMode {
private final MecanumDriveController mecanumDriveController = new MecanumDriveController();
private final PixelMoverController pixelMoverController = new PixelMoverController();
private final List<Controller> controllers = Arrays.asList(mecanumDriveController, pixelMoverController);

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        for (Controller controller : controllers) {
            controller.init(hardwareMap);
        }

        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        pixelMoverController.toggleState(gamepad1, telemetry);
        mecanumDriveController.execute(gamepad1, telemetry);
    }
}
