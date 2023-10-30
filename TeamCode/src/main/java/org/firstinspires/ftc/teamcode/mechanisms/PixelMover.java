package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Implements the intake mechanism for the Robot.
 */
@Config
public class PixelMover implements Mechanism {
    private final String deviceName;
    private final String description;
    //private CRServo brushRoller;
    private CRServo containerRoller;
    private Servo containerBackPush;
    private Servo containerMiddleLock;
    public static double STOPPED_POSITION = 0.5d;
    public static double BACK_STOPPED_POSITION = 0.0d;
    public static double BACK_PUSHED_POSITION = 0.125d;

    public static double MIDDLE_STOPPED_POSITION = 0.0d;
    public static double MIDDLE_LOCKED_POSITION = 0.75d;
    public static double STOPPED_POWER = 0.0d;
    public static double FORWARD_POWER = 0.5d;
    public static double REVERSE_POWER = -0.5d;
    private static enum PixelMoverState {
        PICKING_UP,
        DROPPING_OFF_LOCKED,
        DROPPING_OFF_UNLOCKED,
        STOPPED
    }
    private PixelMoverState state = PixelMoverState.STOPPED;

    public PixelMover(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    @Override
    public void init(HardwareMap hwMap) {
     //   brushRoller = hwMap.get(CRServo.class, "brushRoller");
      //  initServo(brushRoller);
        containerRoller = hwMap.get(CRServo.class, "containerRoller");
        initServo(containerRoller);
        containerBackPush = hwMap.get(Servo.class, "containerBackPush");
        initServo(containerBackPush);
        containerMiddleLock = hwMap.get(Servo.class, "containerMiddleLock");
        initServo(containerMiddleLock);
    }

    /**
     * initServo initializes a continuous rotation servo.
     * @param servo the continuous rotation servo to initializle.
     */
    private void initServo(CRServo servo) {
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPower(STOPPED_POWER);
    }

    /**
     * initServo initializes a position servo.
     * @param servo the position servo to initialize.
     */
    private void initServo(Servo servo) {
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(STOPPED_POSITION);
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
     * Picks up pixel.
     *
     * The first time the A button is pressed, the pixel mover starts to pickup the pixels.  The
     * second time the button is pressed, the bottom pixel is locked and pickup of pixels stops.
     *
     */
    public void pickUpPixels() {
        if (PixelMoverState.STOPPED.equals(state)) {
            // Pixel mover is stopped, get it moving forward
           // brushRoller.setPower(FORWARD_POWER);
            containerRoller.setPower(1.5 * REVERSE_POWER);
           // containerBackPush.setPosition(BACK_STOPPED_POSITION); not needed
            state = PixelMoverState.PICKING_UP;
        }
        else if (PixelMoverState.PICKING_UP.equals(state)) {
            // Pixel mover is rotating forward (clockwise), stop it.
           // brushRoller.setPower(STOPPED_POWER);
            containerRoller.setPower(STOPPED_POWER);
          //  containerBackPush.setPosition(BACK_STOPPED_POSITION); not needed
            containerMiddleLock.setPosition(MIDDLE_LOCKED_POSITION);
            state = PixelMoverState.STOPPED;
        }
    }

    /**
     * Drops off the pixels.
     *
     * The first time the B button is pressed, the pixel mover starts droppin off pixels.  The
     * top pixel should be dropped at this time.  The second time the button is pressed, the bottom
     * pixel is unlocked and is pushed out.  The third time the button is pressed, the servos are
     * reset to their original states.
     *
     */
    public void dropOffPixels() {
        if (PixelMoverState.STOPPED.equals(state)) {
            // Pixel mover is stopped, get it moving forward
            //brushRoller.setPower(REVERSE_POWER);
            containerRoller.setPower(1.5 * FORWARD_POWER); //this should get the top pixel out
            //containerBackPush.setPosition(BACK_PUSHED_POSITION); not needed
            state = PixelMoverState.DROPPING_OFF_LOCKED;
        }
        else if (PixelMoverState.DROPPING_OFF_LOCKED.equals(state)) {
            // Pixel mover is rotating forward (clockwise), stop it.
           // brushRoller.setPower(STOPPED_POWER);
           // containerRoller.setPower(STOPPED_POWER); not needed
            containerMiddleLock.setPosition(MIDDLE_STOPPED_POSITION); //this unlocks the bottom pixel
            containerBackPush.setPosition(BACK_PUSHED_POSITION); //this pushes the bottom pixel
            state = PixelMoverState.DROPPING_OFF_UNLOCKED;
        }
        else if (PixelMoverState.DROPPING_OFF_UNLOCKED.equals(state)) {
            //this resets all the servos in the containers
            containerBackPush.setPosition(BACK_STOPPED_POSITION);
            containerRoller.setPower(STOPPED_POWER);
            state = PixelMoverState.STOPPED;
        }
    }

    @Override
    public String toString() {
        return string();
    }
}
