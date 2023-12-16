package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tests.Test;

import java.util.Collection;

/**
 * Implements the intake mechanism for the Robot.
 */
@Config
public class PixelMover implements Mechanism {
    private final String deviceName;
    private final String description;
    private CRServoImplEx brushRoller;
    private CRServoImplEx containerRoller;
    private Servo containerBackPusher;
    private Servo containerMiddleLock;
    public static double BACK_PUSHER_STOPPED_POSITION = 0.6;
    public static double BACK_PUSHER_PUSHED_POSITION = 0.25;
    public static double MIDDLE_LOCK_STOPPED_POSITION = 0.5;
    public static double MIDDLE_LOCK_LOCKED_POSITION = 0.25;
    public static double STOPPED_POWER = 0.0;
    public static double CONTAINER_ROLLER_FORWARD_POWER = 1.0;
    public static double CONTAINER_ROLLER_REVERSE_POWER = -1.0;
    public static double BRUSH_ROLLER_FORWARD_POWER = 0.5;
    public static double BRUSH_ROLLER_REVERSE_POWER = -0.5;
    private enum PixelMoverState {
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
    public void init(@NonNull HardwareMap hwMap) {
        brushRoller = hwMap.get(CRServoImplEx.class, "brushRoller");
        initServo(brushRoller);
        containerRoller = hwMap.get(CRServoImplEx.class, "containerRoller");
        initServo(containerRoller);
        containerBackPusher = hwMap.get(Servo.class, "containerBackPusher");
        initServo(containerBackPusher, BACK_PUSHER_STOPPED_POSITION);
        containerMiddleLock = hwMap.get(Servo.class, "containerMiddleLock");
        initServo(containerMiddleLock, MIDDLE_LOCK_STOPPED_POSITION);
    }

    /**
     * initServo initializes a continuous rotation servo.
     *
     * @param servo the continuous rotation servo to initialize.
     */
    private void initServo(@NonNull CRServoImplEx servo) {
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPower(STOPPED_POWER);
        servo.setPwmDisable();
    }

    /**
     * initServo initializes a position servo.
     *
     * @param servo the position servo to initialize.
     * @param stoppedPosition the stopped position for the servo
     */
    private void initServo(@NonNull Servo servo, double stoppedPosition) {
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(stoppedPosition);
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
     * Do whatever is needed to be able to start picking up pixels.
     *
     * Things that need to be done so that the robot can pickup pixels.
     *
     *   - Drop the brush roller.
     */
    public void start(Telemetry telemetry) {
        brushRoller.setPower(BRUSH_ROLLER_FORWARD_POWER);
        try {
            Thread.sleep(250);
        } catch (Exception e) {
            telemetry.addLine("PixelMover:  Exception thrown trying to do sleep in start method.");
        }
        brushRoller.setPower(STOPPED_POWER);
    }

    /**
     * Picks up pixels.
     *
     * The first time the A button is pressed, the pixel mover starts to pickup the pixels.  The
     * second time the button is pressed, the bottom pixel is locked and pickup of pixels stops.
     *
     */
    public void pickUpPixels() {
        if (PixelMoverState.STOPPED.equals(state)) {
            // Pixel mover is stopped.
            // Start picking up pixels.
            brushRoller.setPower(BRUSH_ROLLER_REVERSE_POWER);
            containerRoller.setPower(CONTAINER_ROLLER_REVERSE_POWER);
            state = PixelMoverState.PICKING_UP;
        }
        else if (PixelMoverState.PICKING_UP.equals(state)) {
            // Pixel mover is currently picking up pixels.
            // Stop picking up pixels.
            brushRoller.setPower(STOPPED_POWER);
            containerRoller.setPower(STOPPED_POWER);
            containerRoller.setPwmDisable();
            // Lock bottom pixel.
            containerMiddleLock.setPosition(MIDDLE_LOCK_LOCKED_POSITION);
            state = PixelMoverState.STOPPED;
        }
    }

    /**
     * Drops off the pixels.
     *
     * The first time the B button is pressed, the pixel mover starts dropping off pixels.
     * The top pixel should be dropped at this time.  The second time the button is pressed,
     * the bottom pixel is unlocked and is pushed out.  The third time the button is pressed,
     * the pixel mover is reset and is ready to pick up pixels again.
     *
     */
    public void dropOffPixels() {
        if (PixelMoverState.STOPPED.equals(state)) {
            // Pixel mover is stopped, start dropping off pixels
            brushRoller.setPower(BRUSH_ROLLER_FORWARD_POWER);
            containerRoller.setPower(CONTAINER_ROLLER_FORWARD_POWER); //this should get the top pixel out
            state = PixelMoverState.DROPPING_OFF_LOCKED;
        }
        else if (PixelMoverState.DROPPING_OFF_LOCKED.equals(state)) {
            // Unlock the bottom pixel.
            containerMiddleLock.setPosition(MIDDLE_LOCK_STOPPED_POSITION);
            // Push bottom pixel toward container roller to drop it off
            containerBackPusher.setPosition(BACK_PUSHER_PUSHED_POSITION);
            state = PixelMoverState.DROPPING_OFF_UNLOCKED;
        }
        else if (PixelMoverState.DROPPING_OFF_UNLOCKED.equals(state)) {
            // Reset the pixel mover.  This sets all the positional servos to the
            // position they need to be at to begin picking up pixels again; and
            // all of the continuous rotation servos to have no power.
            containerBackPusher.setPosition(BACK_PUSHER_STOPPED_POSITION);
            brushRoller.setPower(STOPPED_POWER);
            containerRoller.setPower(STOPPED_POWER);
            containerRoller.setPwmDisable();
            state = PixelMoverState.STOPPED;
        }
    }
    @Override
    public String toString() {
        return string();
    }
}
