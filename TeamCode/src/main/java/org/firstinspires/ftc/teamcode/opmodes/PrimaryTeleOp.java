package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controls.MecanumDriveController;

@TeleOp(name = "Primary TeleOp", group = "Active")
public class PrimaryTeleOp extends BaseOpMode {
private final MecanumDriveController mecanumDriveController = new MecanumDriveController(robot);

    @Override
    public void loop() {
        mecanumDriveController.execute(gamepad1, telemetry);
    }
}
