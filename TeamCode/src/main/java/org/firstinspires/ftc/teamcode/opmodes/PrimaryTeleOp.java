package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends BaseOpMode {
private final MecanumDriveController mecanumDriveController = new MecanumDriveController(robot);
private final PixelMoverController pixelMoverController = new PixelMoverController(robot);

    @Override
    public void loop() {
        pixelMoverController.toggleState(gamepad1, telemetry);
        mecanumDriveController.execute(gamepad1, telemetry);
    }
}
