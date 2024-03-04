package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.feedback.FeedForward;
import org.firstinspires.ftc.teamcode.feedback.PIDControllerEx;

@Config
@TeleOp(name = "Arm PID Tuning", group = "tuning")
public class ArmTuning extends LinearOpMode {
    public static double TICKS_PER_REV = 383.6; // GoBilda 5203 Yellow Jacket Motor (13.7:1 Ratio)
    public static double ARM_ANGLE_OFFSET = 15; // Degrees offset so arm is parallel to the ground
    private final static double TICKS_IN_DEGREES = TICKS_PER_REV / 360.0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FeedForward ff = p -> Math.cos(Math.toRadians(p / TICKS_IN_DEGREES)) * Kf;
        PIDControllerEx controller = new PIDControllerEx(Kp, Ki, Kd, ff);

        telemetry.addLine("Initialization Complete");

        waitForStart();

        while (opModeIsActive()) {
            // Reset the PID values
            ff = p -> Math.cos(Math.toRadians((p + ARM_ANGLE_OFFSET) / TICKS_IN_DEGREES)) * Kf;
            controller.setPID(Kp, Ki, Kd, ff);

            // Get the arm PID and add in the feed-forward values
            int pos = armMotor.getCurrentPosition();
            double pid = controller.calculate(pos, target);
            armMotor.setPower(pid);

            telemetry.addData("pos", pos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
