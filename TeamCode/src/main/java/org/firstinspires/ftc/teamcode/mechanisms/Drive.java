package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Arrays;
import java.util.Collection;

/**
 * Drive implements the drive chassis for the Robot.
 */
public class Drive implements Mechanism {
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

    public void setMotorsFromCoordinates(double x1, double y1, double turn)
    {
        //Convert cartesian coordinates to polar coordinates
        double headingPower = Math.hypot(-x1, -y1);
        double headingAngle = Math.atan2(-x1, -y1) + (Math.PI / 4);

        double leftFrontPower = (headingPower * Math.cos(headingAngle) + turn);
        double rightFrontPower = (headingPower * Math.sin(headingAngle) - turn);
        double leftRearPower = (headingPower * Math.sin(headingAngle) + turn);
        double rightRearPower = (headingPower * Math.cos(headingAngle) - turn);

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
