package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;

public class TestClaw extends Test{
    private final Claw claw;

    public TestClaw(Claw claw) {
        super(claw.description);
        this.claw = claw;
    }

    @Override
    public void run(Gamepad gamepad1, Telemetry telemetry) {
        if (gamepad1.a) {
            claw.grab();
            telemetry.addData(claw.description, "Grabbed");
        } else {
            claw.release();
            telemetry.addData(claw.description, "Released");
        }
    }
}