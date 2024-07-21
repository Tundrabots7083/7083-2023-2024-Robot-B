package org.firstinspires.ftc.teamcode.mechanism;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.feedback.PIDController;
import org.firstinspires.ftc.teamcode.feedback.PIDControllerEx;

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
    public static double ARM_INTAKE_POSITION_ERROR = 10;
    public static double LIFT_INTAKE_POSITION_ERROR = 10;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final DcMotorEx armMotor;
    private final PIDController leftController;
    private final PIDController rightController;
    private final PIDController armController;
    private final Telemetry telemetry;
    private MotionProfile liftProfile;
    private MotionProfile armProfile;
    private Position targetPosition = Position.INTAKE;

    /**
     * Creates a new Lift hardware mechanism that controls both the two lift motors and the arm
     * motor.
     *
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
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

        leftController = new PIDControllerEx(LIFT_KP, LIFT_KI, LIFT_KD, -LIFT_KG);
        rightController = new PIDControllerEx(LIFT_KP, LIFT_KI, LIFT_KD, -LIFT_KG);
        armController = new PIDController(ARM_KP, ARM_KI, ARM_KD);

        leftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        rightController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        liftProfile = new MotionProfile(LIFT_MAX_ACCELERATION, LIFT_MAX_VELOCITY, 0, Position.INTAKE.liftPosition);
        armProfile = new MotionProfile(ARM_MAX_ACCELERATION, ARM_MAX_VELOCITY, 0, Position.INTAKE.armPosition);
    }

    public void setTarget(Position position) {
        if (targetPosition != position) {
            targetPosition = position;
            telemetry.addData("[LIFT] Set Position", targetPosition.liftPosition);
            telemetry.addData("[ARM] Set Position", targetPosition.armPosition);

            // Build the motion profile to move the lift to the target position
            liftProfile = new MotionProfile(LIFT_MAX_ACCELERATION, LIFT_MAX_VELOCITY, leftMotor.getCurrentPosition(), position.liftPosition);
            armProfile = new MotionProfile(ARM_MAX_ACCELERATION, ARM_MAX_VELOCITY, armMotor.getCurrentPosition(), position.armPosition);

            // Reset the PID controllers
            leftController.reset();
            rightController.reset();
            armController.reset();
        }
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

    @Override
    public void execute() {
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

        // Get the error between the two positions
        double error = Math.abs(leftPosition - targetPosition);

        // TODO: this is wrong. It needs to be "close" to the intake position and then cut off power
        double leftPower, rightPower;
        if (this.targetPosition == Position.INTAKE && error < LIFT_INTAKE_POSITION_ERROR) {
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

        // Get the error between the two positions
        double error = Math.abs(armPosition - targetPosition);

        double armPower;
        if (this.targetPosition == Position.INTAKE && error < ARM_INTAKE_POSITION_ERROR) {
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

    public Action setLiftTo(Position position) {
        return new SetLiftPositionAction(this, position);
    }

    public enum Position {
        INTAKE(0, 0),
        AUTONOMOUS_BACKSTAGE(-2700, -215),
        AUTONOMOUS_FRONTSTAGE(-2700, -350),
        SCORE_LOW(-2700, -350),
        SCORE_MEDIUM(-2700, -700),
        SCORE_HIGH(-2700, -1100),
        HANG_START(0, -1525),
        HANG_END(0, -900),
        LAUNCH_DRONE(0, 0);

        private final double armPosition;
        private final double liftPosition;

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

    /**
     * Sets the lift to the requested position
     */
    private static class SetLiftPositionAction implements Action {

        private final Lift lift;
        private final Position position;

        /**
         * Creates a new action to set the lift to the requested position.
         *
         * @param lift     the lift to move to the requested position
         * @param position the position to which to move the lift
         */
        public SetLiftPositionAction(Lift lift, Position position) {
            this.lift = lift;
            this.position = position;
        }

        /**
         * Moves the lift to the target position. Continues to be called until the lift has
         * reached the target position.
         *
         * @param telemetryPacket telemetry for the action.
         * @return <code>true</code> if the lift has not reached the target position;
         * <cocde>false</cocde> if the lift is at the target position.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            lift.setTarget(position);
            lift.execute();
            return !lift.isAtTarget();
        }
    }
}
