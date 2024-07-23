package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * An extended version of the FTCLib CRServo class that includes pulse width modulation (PWM)
 * capabilities.
 */
public class CRServoEx extends CRServo implements PwmControl {
    /**
     * The constructor for the CR Servo.
     *
     * @param hardwareMap the mapping of all hardware on the robot
     * @param id          the device name for the servo
     */
    public CRServoEx(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id);

        // Override the CRServo implementation in the superclass with one with PWM management support
        crServo = hardwareMap.get(CRServoImplEx.class, id);
    }

    /**
     * Gets the PWM range limits for the servo
     *
     * @return the PWM range limits for the servo
     */
    @Override
    public PwmRange getPwmRange() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        return crServoEx.getPwmRange();
    }

    /**
     * Sets the PWM range limits for the servo.
     *
     * @param range the new PWM range limits for the servo
     */
    @Override
    public void setPwmRange(PwmRange range) {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        crServoEx.setPwmRange(range);
    }

    /**
     * Enables PWM for the servo.
     */
    @Override
    public void setPwmEnable() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        crServoEx.setPwmEnable();
    }

    /**
     * Disables PWM for the servo.
     */
    @Override
    public void setPwmDisable() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        crServoEx.setPwmDisable();
    }

    /**
     * Gets indication as to whether PWM is enabled for the servo.
     *
     * @return <code>true</code> if PWM is enabled; <code>false</code> if PWM is not enabled.
     */
    @Override
    public boolean isPwmEnabled() {
        CRServoImplEx crServoEx = (CRServoImplEx) crServo;
        return crServoEx.isPwmEnabled();
    }
}
