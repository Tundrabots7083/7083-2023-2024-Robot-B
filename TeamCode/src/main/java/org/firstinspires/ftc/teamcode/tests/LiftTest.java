package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;

public class LiftTest extends Test {
    private final Lift lift;
    public LiftTest(Lift lift) {
        super("Lift Test");

        this.lift = lift;
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a) {
            lift.overrideLiftPower(0.5);
        } else if (gamepad1.b) {
            lift.overrideLiftPower(-0.5);
        } else if (gamepad1.x) {
            lift.overrideArmPower(0.5);
        } else if (gamepad1.y) {
            lift.overrideArmPower(-0.5);
        }
    }
}
