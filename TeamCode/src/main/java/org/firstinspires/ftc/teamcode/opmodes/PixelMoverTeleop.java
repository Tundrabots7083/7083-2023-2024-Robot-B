package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TestRobot;
import org.firstinspires.ftc.teamcode.controls.PixelMoverController;
@TeleOp (name="Pixel Mover TeleOp", group="test")
public class PixelMoverTeleop extends OpMode {
    private final Robot robot = new TestRobot();
    private final PixelMoverController pixelMoverController = new PixelMoverController(robot);

    @Override
    public void init() {
        robot.pixelMover.init(hardwareMap);
    }

    @Override
    public void loop() {
        pixelMoverController.toggleState(gamepad1, telemetry);
    }
}
