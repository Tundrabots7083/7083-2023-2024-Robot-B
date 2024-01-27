package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.feedback.PIDCoefficients;
import org.firstinspires.ftc.teamcode.feedback.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Arm is the arm and pixel container that are used for pixel intake and pixel scoring.
 */
@Config
public class Lift implements Mechanism {


    public static double LIFT_KP = 0.0053;
    public static double LIFT_KI = 0.0;
    public static double LIFT_KD = 0.0;
    public static double INTEGRAL_LIMIT = 1;
    public static double LIFT_MAX_ACCELERATION = 0.5;
    public static double LIFT_MAX_VELOCITY = 0.5;
    public static double MINIMUM_LIFT_POWER = 0.1;

    public static double ARM_KP = 0.0053;
    public static double ARM_KI = 0.0;
    public static double ARM_KD = 0.0;
    public static double ARM_MAX_ACCELERATION = 0.5;
    public static double ARM_MAX_VELOCITY = 0.5;
    public static double MINIMUM_ARM_POWER = 0.1;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx armMotor;

    PIDController leftController;
    PIDController rightController;
    PIDController armController;

    MotionProfile liftProfile;
    MotionProfile armProfile;

    Position targetPosition;

    Telemetry telemetry;

    @Override
    public String getDeviceName() {
        return "Lift";
    }

    @Override
    public String getDescription() {
        return "The lift is used to lift the robot off the ground and to score pixels.";
    }

    @Override
    public Collection<Test> getTests() {
        return null;
    }

    public enum Position {
        Start(0, 0),
        Intake(0, 0),
        ScoreLow(2900, 0),
        ScoreMedium(2750, 0),
        ScoreHigh(2500, 0),
        Hang(2000, 0),
        LaunchDrone(1455, 0);

        public final int armPosition;
        public final int liftPosition;

        /**
         * Creates a new Position for the given arm and servo.
         * @param armPosition the position of the arm.
         * @param liftPosition the position of the lift.
         */
        Position(int armPosition, int liftPosition) {
            this.armPosition = armPosition;
            this.liftPosition = liftPosition;
        }
    }

    public Lift(HardwareMap hardwareMap) {
        this.leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftController = new PIDController(new PIDCoefficients(LIFT_KP, LIFT_KI, LIFT_KD));
        rightController = new PIDController(new PIDCoefficients(LIFT_KP, LIFT_KI, LIFT_KD));
        armController = new PIDController(new PIDCoefficients(ARM_KP, ARM_KI, ARM_KD));

        leftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        rightController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        liftProfile = new MotionProfile(LIFT_MAX_ACCELERATION, LIFT_MAX_VELOCITY, 0, Position.Start.liftPosition);
    }

    public void setTarget(Position position) {
        this.targetPosition = position;

        // Build the motion profile to move the lift to the target position
        liftProfile = new MotionProfile(LIFT_MAX_ACCELERATION, LIFT_MAX_VELOCITY, leftMotor.getCurrentPosition(), position.liftPosition);

        // Reset the PID controllers
        leftController.reset();
        rightController.reset();
    }

    public void update() {
        // Update the power level for the arm motor
        setArmPower();
        // Update power level for the lift motors
        setLiftPower();
    }

    private void setLiftPower() {
        // Read the current position of each motor
        double leftPosition = leftMotor.getCurrentPosition();
        double rightPosition = rightMotor.getCurrentPosition();

        // Get the target position from our motion profile
        double targetPosition = liftProfile.calculatePosition();

        // Calculate the power for each motor using the PID controllers
        double leftPower = leftController.calculate(targetPosition, leftPosition);
        double rightPower = rightController.calculate(targetPosition, rightPosition);

        // Cap the motor power at 1 and -1
        leftPower = modifyMotorPower(leftPower, MINIMUM_LIFT_POWER);
        rightPower = modifyMotorPower(rightPower, MINIMUM_LIFT_POWER);

        // Apply the power to each motor
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        String leftMotorTelemetry = "power: " + leftPower + ", position: " + leftPosition;
        String rightMotorTelemetry = "power: " + rightPower + ", position: " + rightPosition;
        telemetry.addData("[LIFT] target", targetPosition);
        telemetry.addData("[LIFT] left", leftMotorTelemetry);
        telemetry.addData("[LIFT] right", rightMotorTelemetry);
    }

    private void setArmPower() {
        // Read the current position of the arm motor
        double armPosition = armMotor.getCurrentPosition();

        // Get the target position from our motion profile
        double targetPosition = armProfile.calculatePosition();

        // Calculate the power for the arm motor using the PID controller
        double armPower = armController.calculate(targetPosition, armPosition);

        // Cap the motor power at 1 and -1
        armPower = modifyMotorPower(armPower, MINIMUM_LIFT_POWER);

        // Apply the power to the arm motor
        armMotor.setPower(armPower);

        telemetry.addData("[ARM] target", targetPosition);
        telemetry.addData("[ARM] power", armPower);
        telemetry.addData("[ARM] position", armPosition);
    }

    private double modifyMotorPower(double power, double minPower) {
        // Cap the motor power at 1 and -1
        power = Math.max(-1, Math.min(1, power));
        // If the power level of the motor is below the minimum threshold, set it to 0
        if (Math.abs(power) < minPower) {
            power = 0;
        }
        return power;
    }
}
