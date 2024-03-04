package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.feedback.FeedForwardFun;
import org.firstinspires.ftc.teamcode.feedback.PIDControllerEx;

@Config
@TeleOp(name = "Lift PID Tuning", group = "tuning")
public class LiftTuning extends LinearOpMode {
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDControllerEx leftLiftController = new PIDControllerEx(Kp, Ki, Kd, Kf);
        PIDControllerEx rightLiftController = new PIDControllerEx(Kp, Ki, Kd, Kf);

        telemetry.addLine("Initialization Complete");

        waitForStart();

        while (opModeIsActive()) {
            // Reset the PID values
            leftLiftController.setPID(Kp, Ki, Kd, Kf);
            rightLiftController.setPID(Kp, Ki, Kd, Kf);

            // Get the left and right motor power outputs returned by the PID controller
            int leftLiftPos = leftMotor.getCurrentPosition();
            double leftMotorPid = leftLiftController.calculate(leftLiftPos, target);
            int rightLiftPos = leftMotor.getCurrentPosition();
            double rightMotorPid = leftLiftController.calculate(rightLiftPos, target);

            // Set the motor powers
            leftMotor.setPower(leftMotorPid);
            rightMotor.setPower(rightMotorPid);

            telemetry.addData ("Left Lift", leftMotorPid);
            telemetry.addData ("Right Lift", rightMotorPid);
            telemetry.addData ("Target", target);
            telemetry.update();
        }
    }
}
