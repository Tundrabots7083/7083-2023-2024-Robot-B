package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.tests.LiftTest;
import org.firstinspires.ftc.teamcode.tests.MecanumDriveTest;

@TeleOp(name = "MecanumDrive Test", group = "test")
public class TestMecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = Robot.getInstance(hardwareMap, telemetry);
        MecanumDrive mecanumDrive = robot.mecanumDrive;
        MecanumDriveTest test = new MecanumDriveTest(mecanumDrive);

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            test.run(gamepad1, gamepad2);
        }
    }
}
