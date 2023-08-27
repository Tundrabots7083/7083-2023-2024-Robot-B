package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Motor is a motor used on the Robot. It provides a place to set all the various configurations
 * needed for the motor being used.
 */
public class Motor implements Mechanism {
    private static final boolean RUN_USING_ENCODER = false;

    private final String deviceName;
    private final String description;
    private DcMotorEx motor;
    private DcMotorSimple.Direction direction;
    private double ticksPerRotation;

    /**
     * Motor is a motor found on the Robot. The motor may be used for turning wheels, operating
     * pulleys, and various other tasks on the Robot.
     *
     * @param deviceName the name of the motor, as configured on the control hub.
     * @param description a description of the motor.
     */
    public Motor(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    @Override
    public void init(HardwareMap hwMap) {
        motor = (DcMotorEx) hwMap.dcMotor.get(deviceName);

        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        ticksPerRotation = motor.getMotorType().getTicksPerRev();
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
     * getCurrentPosition returns the current position of the motor.
     * @return the current position of the notor.
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * getMotorRotations returns the number of rotations a motor has experienced.
     *
     * @return the number of rotations for the motor.
     */
    public double getMotorRotations() {
        return getCurrentPosition() / getTicksPerRotation();
    }

    /**
     * getTicksPerRotation returns the number of `ticks` that occur for every rotation of
     * the motor.
     *
     * @return the number of ticks per rotation of the motor.
     */
    public double getTicksPerRotation() {
        return ticksPerRotation;
    }

    /**
     * setPower sets the current to be applied to the motor.
     * @param power the amount of power to apply to the motor.
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     * setDirection specifies whether the motor should operate in the forward or reverse direction.
     * This is used to adjust for a motor being on the right or left side of the Robot.
     *
     * @param direction the direction the motor operates in.
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public String toString() {
        return string();
    }
}
