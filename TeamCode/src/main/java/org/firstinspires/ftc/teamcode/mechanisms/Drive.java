package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

/**
 * Drive implements the drive chassis for the Robot.
 */
public class Drive implements Mechanism {

    private static final double MM_PER_IN = 25.4;

    private static final double WHEEL_DIAM_IN = 96 / MM_PER_IN; // for a Mecanum wheel

    private final String deviceName;
    private final String description;
    private Motor rightFront, rightRear, leftFront, leftRear;
    private Collection<Motor> motors;

    public Drive(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftFront = new Motor("leftFront", "Left Front");
        leftRear = new Motor("leftRear", "Left Rear");
        rightFront = new  Motor("rightFront", "Right Front");
        rightRear = new Motor("rightRear", "Right Rear");
        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
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
    public Collection<Motor> getMotors() {
        return motors;
    }

    @Override
    public Collection<Test> getTests() {
        return null;
    }

    /**
     * move sets the powers to the wheel motors to result in the robot moving
     * in the direction provided on input.
     * @param forward how much to move forward or, if a negative value, backward.
     * @param right how much to move right or, if a negative value, left.
     * @param rotate how much to rotate the robot.
     */
    public void move(double forward, double right, double rotate) {
        double leftFrontPower = forward + right + rotate;
        double leftRearPower = forward - right + rotate;
        double rightFrontPower = forward - right - rotate;
        double rightRearPower = forward + right - rotate;

        setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    public void setMotorsFromCoordinates(double x1, double y1, double turn)
    {
        //Convert cartesian coordinates to polar coordinates
        double headingPower = Math.hypot(-x1, -y1);
        double headingAngle = Math.atan2(-x1, -y1) + (Math.PI / 4);

        double leftFrontPower = (headingPower * Math.cos(headingAngle) + turn);
        double rightFrontPower = (headingPower * Math.sin(headingAngle) - turn);
        double leftRearPower = (headingPower * Math.sin(headingAngle) + turn);
        double rightRearPower = (headingPower * Math.cos(headingAngle) - turn);

        setMotorPowers(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    /**
     * setMotorPowers sets the power for the wheels, normalizing for a maximum power of 1.0.
     * @param leftFrontPower the power for the left front motor.
     * @param leftRearPower the power for the left rear motor
     * @param rightFrontPower the power for the right front motor.
     * @param rightRearPower the power for the right rear motor.
     */
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
        double maxPower = 1.0;

        // Get the maximum power for any motor, or 1.0, whichever is greater
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

        wheelPositions.add(ticksToInches(leftFront.getCurrentPosition(), leftFront.getTicksPerRotation()));
        wheelPositions.add(ticksToInches(leftRear.getCurrentPosition(), leftRear.getTicksPerRotation()));
        wheelPositions.add(ticksToInches(rightFront.getCurrentPosition(), rightFront.getTicksPerRotation()));
        wheelPositions.add(ticksToInches(rightRear.getCurrentPosition(), rightRear.getTicksPerRotation()));

        return wheelPositions;
    }

    @Override
    public String toString() {
        return string();
    }
}
