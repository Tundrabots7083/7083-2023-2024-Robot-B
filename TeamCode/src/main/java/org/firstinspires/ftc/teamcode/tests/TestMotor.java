package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Motor;

/**
 * TestMotor contains a basic test of the motor
 */
public class TestMotor extends Test {
    private final Motor motor;
    private final double speed;

    public TestMotor(Motor motor, String description, double speed) {
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