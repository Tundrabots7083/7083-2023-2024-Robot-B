package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Arrays;
import java.util.Collection;

/**
 * Drive implements the drive chassis for the Robot.
 */
public class MecanumDrive implements Mechanism {
    private final String deviceName;
    private final String description;
    private DcMotorEx rightFront, rightRear, leftFront, leftRear;
    private Collection<DcMotorEx> motors;

    /**
     * MecanumDrive initializes a new mecanum drive trail.
     * @param deviceName  the name of the mecanum drive.
     * @param description a description of the mecanum drive.
     */
    public MecanumDrive(String deviceName, String description, HardwareMap hardwareMap) {
        this.deviceName = deviceName;
        this.description = description;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
            initMotor(motor);
        }
    }

    /**
     * initMotor initializes a motor attached to the mecanum wheel.
     * @param motor the motor to be initialized.
     */
    private void initMotor(DcMotorEx motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    @Override
    public String getDescription() {
        return description;
    }

    @Override
    public Collection<Test> getTests() {
        return null;
    }

    /**
     * drive sets the powers to the wheel motors to result in the robot moving
     * in the direction provided on input.
     *
     * @param x how much to move right or, if a negative value, left.
     * @param y how much to move forward or, if a negative value, backward.
     * @param turn how much to rotate the robot.
     */
    public void drive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos/max - turn;
        double rightFrontPower = power * sin/max + turn;
        double leftRearPower = power * sin/max - turn;
        double rightRearPower = power * cos/max + turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + turn;
            rightFrontPower /= power + turn;
            leftRearPower /= power + turn;
            rightRearPower /= power + turn;
        }
        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * setMotorPowers sets the power for the wheels, normalizing for a maximum power of 1.0.
     * @param leftFrontPower the power for the left front motor.
     * @param leftRearPower the power for the left rear motor
     * @param rightRearPower the power for the right rear motor.
     * @param rightFrontPower the power for the right front motor.
     */
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
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

    @Override
    public String toString() {
        return string();
    }
}
