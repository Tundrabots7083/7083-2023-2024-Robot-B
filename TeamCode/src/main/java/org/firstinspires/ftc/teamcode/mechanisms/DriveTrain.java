package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

/**
 * Drive implements the drive chassis for the Robot.
 */
public class DriveTrain implements Mechanism {
    private static final boolean RUN_USING_ENCODER = false;

    private static final double MM_PER_IN = 25.4;

    private static final double WHEEL_DIAM_IN = 96 / MM_PER_IN; // for a Mecanum wheel

    private final String deviceName;
    private final String description;
    private final Robot robot;
    private DcMotorEx rightFront, rightRear, leftFront, leftRear;
    private Collection<DcMotorEx> motors;

    public DriveTrain(Robot robot, String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
        this.robot = robot;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
            initMotor(motor);
        }
    }

    private void initMotor(DcMotorEx motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    @Override
    public String getDescription() {
        return description;
    }

    /**
     * getMotors returns a collection of motors used by this drive.
     *
     * @return a collection of motors used by this drive.
     */
    public Collection<DcMotorEx> getMotors() {
        return motors;
    }

    @Override
    public Collection<Test> getTests() {
        return null;
    }

    /**
     * drive sets the powers to the wheel motors to result in the robot moving
     * in the direction provided on input.
     * @param forward how much to move forward or, if a negative value, backward.
     * @param right how much to move right or, if a negative value, left.
     * @param rotate how much to rotate the robot.
     */
    public void drive(double forward, double right, double rotate) {
        double leftFrontPower = forward + right + rotate;
        double leftRearPower = forward - right + rotate;
        double rightFrontPower = forward - right - rotate;
        double rightRearPower = forward + right - rotate;

        setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    /**
     * setMotorsFromCoordinates takes the values from the gamepad and converts
     * them to the proper power to apply to the motors to accomplish the behavior
     * @param forward the x value of the left joystick.
     * @param right the y value of the left joystick.
     * @param rotate the x value of the right joystick
     */
    public void setMotorsFromCoordinates(double forward, double right, double rotate) {
        double robotAngle = robot.gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
  
        // Convert cartesian coordinates to polar coordinates
        double distance = Math.hypot(forward, right);
        double angle = Math.atan2(forward, right) + (Math.PI / 4);

        // Rotate the angle
        angle = AngleUnit.normalizeRadians(angle - robotAngle);

        // Convert back to cartesian coordinates
        double newForward = distance * Math.sin(angle);
        double newRight = distance * Math.cos(angle);

        // Calculate the power to apply to each motor
        drive(newForward, newRight, rotate);
    }

    /**
     * setMotorPowers sets the power for the wheels, normalizing for a maximum power of 1.0.
     * @param leftFrontPower the power for the left front motor.
     * @param leftRearPower the power for the left rear motor
     * @param rightFrontPower the power for the right front motor.
     * @param rightRearPower the power for the right rear motor.
     */
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
        // Get the maximum power for any motor, or 1.0, whichever is greater
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));

        // Divide by the maximum power, which guarantees no motor's power will exceed 1.0.
        // This also ensures that all motors get a proportional amount of power should the
        // input power for any motor exceed 1.0.
        leftFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightFrontPower /= maxPower;
        rightRearPower /= maxPower;

        // Now that the power have been normalized, go ahead and set power for the motors.
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * ticksToInches converts the position as reported by the motor to a number of inches.
     * @param ticks the position as reported by the motor.
     * @return the number of inches the wheel has moved based on the number of ticks.
     */
    private double ticksToInches(double ticks, double ticksPerRotation) {
        return WHEEL_DIAM_IN * Math.PI * ticks / ticksPerRotation;
    }

    /**
     * getWheelPosition returns a list of positions for each of the four wheels. The order of the
     * list is `left front`, `left rear`, `right front`, right rear`.
     * @return a list of positions for each of the wheels.
     */
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();

        wheelPositions.add(ticksToInches(leftFront.getCurrentPosition(), leftFront.getMotorType().getTicksPerRev()));
        wheelPositions.add(ticksToInches(leftRear.getCurrentPosition(), leftRear.getMotorType().getTicksPerRev()));
        wheelPositions.add(ticksToInches(rightFront.getCurrentPosition(), rightFront.getMotorType().getTicksPerRev()));
        wheelPositions.add(ticksToInches(rightRear.getCurrentPosition(), rightRear.getMotorType().getTicksPerRev()));

        return wheelPositions;
    }

    @Override
    public String toString() {
        return string();
    }
}
