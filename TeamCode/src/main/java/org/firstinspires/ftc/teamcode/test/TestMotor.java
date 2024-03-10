package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TestMotor contains a basic test of the motor
 */
public class TestMotor extends Test {
    private final DcMotorEx motor;
    private final double speed;

    public TestMotor(DcMotorEx motor, String description, double speed) {
        super(description);
        this.speed = speed;
        this.motor = motor;
    }

    @Override
    public void run(Gamepad gamepad1, Telemetry telemetry) {
        if (gamepad1.a) {
            motor.setPower(speed);
        } else {
            motor.setPower(0.0);
        }
        telemetry.addData("Encoder", motor.getCurrentPosition());
    }
}