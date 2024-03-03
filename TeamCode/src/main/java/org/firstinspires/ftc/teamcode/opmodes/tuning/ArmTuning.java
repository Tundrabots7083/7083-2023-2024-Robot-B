package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.feedback.PIDController;

@Config
@TeleOp(name = "Arm PID Tuning", group = "tuning")
public class ArmTuning extends LinearOpMode {
    private final static double TICKS_IN_DEGREES = 700 / 180.0;
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

        PIDController controller = new PIDController(Kp, Ki, Kd);

        telemetry.addLine("Initialization Complete");

        waitForStart();

        while (opModeIsActive()) {
            // Reset the PID values
            controller.setPID(Kp, Ki, Kd);

            // Get the arm PID and add in the feed-forward values
            int pos = armMotor.getCurrentPosition();
            double pid = controller.calculate(pos, target);
            double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES)) * Kf;
            double power = pid + ff;
            armMotor.setPower(power);

            telemetry.addData("pos", pos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
