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
    public static double CONTAINER_ROLLER_STOPPED_POWER = -.08;
    public static double BRUSH_ROLLER_FORWARD_POWER = 0.08;
    public static double BRUSH_ROLLER_REVERSE_POWER = -0.5;
    private enum PixelMoverState {
        PICKING_UP,
        DROPPING_OFF_LOCKED,
        DROPPING_OFF_UNLOCKED,
        STOPPED
    }
    private PixelMoverState state = PixelMoverState.STOPPED;

    public PixelMover(String deviceName, String description, HardwareMap hardwareMap) {
        this.deviceName = deviceName;
        this.description = description;

        brushRoller = hardwareMap.get(CRServoImplEx.class, "brushRoller");
        initServo(brushRoller);
        containerRoller = hardwareMap.get(CRServoImplEx.class, "containerRoller");
        initServo(containerRoller);
        containerBackPusher = hardwareMap.get(Servo.class, "containerBackPusher");
        initServo(containerBackPusher, BACK_PUSHER_STOPPED_POSITION);
        containerMiddleLock = hardwareMap.get(Servo.class, "containerMiddleLock");
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
     * This method is invoked when an autonomous opmode is started.
     *
     * The pixel container is assumed to be pre-loaded with two pixels and the
     * robot is assumed to be in its folded position that fits in an
     * 18X18X18 cube.  The robot needs to be unfolded and prepared to perform
     * an autonomous sequence.
     *
     * Things that need to be done so that the robot can pickup pixels.
     *
     *   - Drop the brush roller.
     *   - Lock the pixels in the container.
     */
    public void start(Telemetry telemetry, boolean lock) {
        brushRoller.setPower(BRUSH_ROLLER_FORWARD_POWER);
        try {
            Thread.sleep(3000);
        } catch (Exception e) {
            telemetry.addLine("PixelMover:  Exception thrown trying to do sleep in start method.");
        }
        brushRoller.setPower(STOPPED_POWER);

        if (lock) {
            lockPixels();
        }
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
            lockPixels();
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

    /**
     * Locks the pixels in the container and sets the state to indicate
     * it is stopped or not currently picking up or dropping off pixels.
     *
     */
    public void lockPixels() {
        containerMiddleLock.setPosition(MIDDLE_LOCK_LOCKED_POSITION);
        containerRoller.setPower(CONTAINER_ROLLER_STOPPED_POWER);
    }

    /**
     * Drops off only the top pixel in the container and leaves
     * the bottom pixel locked.
     *
     * This method assumes there are two pixels in the container.
     * The brush roller and the container roller are started to
     * cause the top pixel to come out of the container and then
     * the rollers are stopped.  The container roller is given a
     * small amount of power to insure that the bottom pixel will
     * not accidentally come out of the container.
     *
     * This method assumes it is invoked from an autonomous opmode
     * and waits for .25 of a second to give the top pixel time
     * to come out of the container.
     *
     * @param telemetry
     */
    public void dropOffTopPixel(Telemetry telemetry) {
        brushRoller.setPower(BRUSH_ROLLER_FORWARD_POWER);
        containerRoller.setPower(CONTAINER_ROLLER_FORWARD_POWER);
        try {
            Thread.sleep(250);
        } catch (Exception e) {
            telemetry.addLine("PixelMover:  Exception thrown trying to do sleep in dropOffTopPixel method.");
        }
        brushRoller.setPower(STOPPED_POWER);
        containerRoller.setPower(CONTAINER_ROLLER_STOPPED_POWER);
    }

    /**
     * Drops off the bottom pixel that is locked in the container.
     *
     * This method assumes that the container has just one pixel in it
     * and that it is locked.
     *
     * This method assumes it is invoked from an autonomous opmode
     * and waits for .25 of a second to give the pixel time to come
     * out of the container.
     *
     * @param telemetry
     */
    public void dropOffBottomPixel(Telemetry telemetry) {
        // Start container roller
        containerRoller.setPower(CONTAINER_ROLLER_FORWARD_POWER);
        // Unlock the pixel
        containerMiddleLock.setPosition(MIDDLE_LOCK_STOPPED_POSITION);
        // Push the pixel toward container roller to drop it off
        containerBackPusher.setPosition(BACK_PUSHER_PUSHED_POSITION);
        // Wait for pixel to come out of container
        try {
            Thread.sleep(250);
        } catch (Exception e) {
            telemetry.addLine("PixelMover:  Exception thrown trying to do sleep in dropOffBottomPixel method.");
        }
        // Container should now be empty, reset the servos so the container can pick up
        // pixels again.
        containerBackPusher.setPosition(BACK_PUSHER_STOPPED_POSITION);
        containerRoller.setPower(STOPPED_POWER);
        containerRoller.setPwmDisable();
    }

    @Override
    public String toString() {
        return string();
    }
}
