package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

public class MecanumDriveTest extends Test {
    private final MecanumDrive mecanumDrive;

    public MecanumDriveTest(MecanumDrive mecanumDrive) {
        super("MecanumDrive Test");

        this.mecanumDrive = mecanumDrive;
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a && !gamepad1.left_bumper) {
            mecanumDrive.leftFront.setPower(1.0);
        } else if (gamepad1.a && gamepad1.left_bumper) {
            mecanumDrive.leftFront.setPower(-1.0);
        } else {
            mecanumDrive.leftFront.setPower(0);
        }

        if (gamepad1.b && !gamepad1.left_bumper) {
            mecanumDrive.leftRear.setPower(1.0);
        } else if (gamepad1.b && gamepad1.left_bumper) {
            mecanumDrive.leftRear.setPower(-1.0);
        } else {
            mecanumDrive.leftRear.setPower(0);
        }

        if (gamepad1.x && !gamepad1.left_bumper) {
            mecanumDrive.rightRear.setPower(1.0);
        } else if (gamepad1.x && gamepad1.left_bumper) {
            mecanumDrive.rightRear.setPower(-1.0);
        } else {
            mecanumDrive.rightRear.setPower(0);
        }

        if (gamepad1.y && !gamepad1.left_bumper) {
            mecanumDrive.rightFront.setPower(1.0);
        } else if (gamepad1.y && gamepad1.left_bumper) {
            mecanumDrive.rightFront.setPower(-1.0);
        } else {
            mecanumDrive.rightFront.setPower(0);
        }
    }
}
