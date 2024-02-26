package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.feedback.PIDCoefficients;
import org.firstinspires.ftc.teamcode.feedback.PIDCoefficientsEx;
import org.firstinspires.ftc.teamcode.feedback.PIDController;
import org.firstinspires.ftc.teamcode.feedback.PIDControllerEx;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Arm is the arm and pixel container that are used for pixel intake and pixel scoring.
 */
@Config
public class Lift implements Mechanism {
    public static double LIFT_KP = 0.009;
    public static double LIFT_KI = 0.0;
    public static double LIFT_KD = 0.0;
    public static double LIFT_KG = 0.1;
    public static double INTEGRAL_LIMIT = 1;
    public static double LIFT_MAX_ACCELERATION = 3000;
    public static double LIFT_MAX_VELOCITY = 8000;
    public static double MINIMUM_LIFT_POWER = 0.1;
    public static double ARM_KP = 0.003;
    public static double ARM_KI = 0.0;
    public static double ARM_KD = 0.0;
    public static double ARM_MAX_ACCELERATION = 3000;
    public static double ARM_MAX_VELOCITY = 5000;
    public static double MINIMUM_ARM_POWER = 0.16;
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

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftController = new PIDControllerEx(new PIDCoefficientsEx(LIFT_KP, LIFT_KI, LIFT_KD, -LIFT_KG));
        rightController = new PIDControllerEx(new PIDCoefficientsEx(LIFT_KP, LIFT_KI, LIFT_KD, -LIFT_KG));
        armController = new PIDController(new PIDCoefficients(ARM_KP, ARM_KI, ARM_KD));

        leftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        rightController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        liftProfile = new MotionProfile(LIFT_MAX_ACCELERATION, LIFT_MAX_VELOCITY, 0, Position.INTAKE.liftPosition);
        armProfile = new MotionProfile(ARM_MAX_ACCELERATION, ARM_MAX_VELOCITY, 0, Position.INTAKE.armPosition);
    }

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

    public void setTarget(Position position) {
        this.targetPosition = position;

        // Build the motion profile to move the lift to the target position
        liftProfile = new MotionProfile(LIFT_MAX_ACCELERATION, LIFT_MAX_VELOCITY, leftMotor.getCurrentPosition(), position.liftPosition);
        armProfile = new MotionProfile(ARM_MAX_ACCELERATION, ARM_MAX_VELOCITY, armMotor.getCurrentPosition(), position.armPosition);

        // Reset the PID controllers
        leftController.reset();
        rightController.reset();
        armController.reset();
    }

    /**
     * Returns indication as to whether the arm is at the target position.
     *
     * @return <code>true</code> if the arm is at the target position;
     * <code>false</code> if the arm is not.
     */
    public boolean isArmAtTarget() {
        return armProfile.isAtEnd();
    }

    /**
     * Returns indication as to whether the lift is at the target position.
     *
     * @return <code>true</code> if the lift is at the target position;
     * <code>false</code> if the lift is not.
     */
    public boolean isLiftAtTarget() {
        return liftProfile.isAtEnd();
    }

    /**
     * Returns indication as to whether the lift and arm are at the target position.
     *
     * @return <code>true</code> if the lift and arm are at the target position;
     * <code>false</code> if either is not.
     */
    public boolean isAtTarget() {
        return isLiftAtTarget() && isArmAtTarget();
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

        double leftPower, rightPower;
        if (targetPosition == Position.INTAKE.liftPosition) {
            leftPower = 0;
            rightPower = 0;
        } else {
            // Calculate the power for each motor using the PID controllers
            leftPower = leftController.calculate(targetPosition, leftPosition);
            rightPower = rightController.calculate(targetPosition, rightPosition);

            // Cap the motor power at 1 and -1
            leftPower = modifyMotorPower(leftPower, MINIMUM_LIFT_POWER);
            rightPower = modifyMotorPower(rightPower, MINIMUM_LIFT_POWER);
        }

        // Apply the power to each motor
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("[LIFT] target", targetPosition);
        telemetry.addData("[LIFT] left position", leftPosition);
        telemetry.addData("[LIFT] left power", leftPower);
        telemetry.addData("[LIFT] right position", rightPosition);
        telemetry.addData("[LIFT] right power", rightPower);
    }

    private void setArmPower() {
        // Read the current position of the arm motor
        int armPosition = armMotor.getCurrentPosition();

        // Get the target position from our motion profile
        double targetPosition = armProfile.calculatePosition();

        double armPower;
        if (targetPosition == Position.INTAKE.armPosition) {
            armPower = 0;
        } else {
            // Calculate the power for the arm motor using the PID controller
            armPower = armController.calculate(targetPosition, armPosition);

            // Cap the motor power at 1 and -1
            armPower = modifyMotorPower(armPower, MINIMUM_ARM_POWER);
        }

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

    public void overrideArmPower(double power) {
        armMotor.setPower(power);
        setArmTelemetry();
    }

    public void overrideLiftPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        setLiftTelemetry();
    }

    private void setArmTelemetry() {
        telemetry.addData("[ARM] position", armMotor.getCurrentPosition());
    }

    private void setLiftTelemetry() {
        telemetry.addData("[LIFT] left power", leftMotor.getPower());
        telemetry.addData("[LIFT] left position", leftMotor.getCurrentPosition());
        telemetry.addData("[LIFT] right power", rightMotor.getPower());
        telemetry.addData("[LIFT] right position", rightMotor.getCurrentPosition());
    }

    public enum Position {
        INTAKE(0, 0),
        AUTONOMOUS_BACKSTAGE(-2700, -200),
        AUTONOMOUS_FRONTSTAGE(-2700, -400),
        SCORE_LOW(-2700, -350),
        SCORE_MEDIUM(-2700, -700),
        SCORE_HIGH(-2700, -1100),
        HANG_START(0, -1525),
        HANG_END(0, -900),
        LAUNCH_DRONE(0, 0);

        public final int armPosition;
        public final int liftPosition;

        /**
         * Creates a new Position for the given arm and servo.
         *
         * @param armPosition  the position of the arm.
         * @param liftPosition the position of the lift.
         */
        Position(int armPosition, int liftPosition) {
            this.armPosition = armPosition;
            this.liftPosition = liftPosition;
        }
    }
}
