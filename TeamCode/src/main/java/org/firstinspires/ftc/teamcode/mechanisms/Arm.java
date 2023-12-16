package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.teamcode.tests.Test;
import org.firstinspires.ftc.teamcode.utils.SizedStack;

import java.util.Collection;

/**
 * Arm is the arm and pixel container that are used for pixel intake and pixel scoring.
 */
@Config
public class Arm implements Mechanism {
    /**
     * Position defines the position of the arm and pixel container.
     */
    public enum Position {
        Hang(1000, 0, 0),
        Intake(0, 0, 0),
        ScoreLow(1500, 0, 0),
        ScoreMedium(2000, 100, 0),
        ScoreHigh(2400, 200, 0),
        Start(800, 0,0.6);

        private final int armPosition;
        private final int liftPosition;
        private final double servoPosition;

        /**
         * Creates a new Position for the given arm and servo.
         * @param armPosition the position of the arm.
         * @param liftPosition the position of the lift.
         * @param servoPosition the position of the flip servo.
         */
        Position(int armPosition, int liftPosition, double servoPosition) {
            this.armPosition = armPosition;
            this.liftPosition = liftPosition;
            this.servoPosition = servoPosition;
        }
    }

    // PIDF control constants. TODO: change to private final once tuned
    public static double ARM_KP = 0.0;
    public static double ARM_KI = 0.0;
    public static double ARM_KD = 0.0;
    public static double ARM_KF = 0.0;
    public static double LIFT_KP = 0.0;
    public static double LIFT_KI = 0.0;
    public static double LIFT_KD = 0.0;
    public static double LIFT_KF = 0.0;
    public static double INTEGRAL_LIMIT = 1;

    // TODO: change to private final once tuned
    public static int TOLERABLE_ERROR = 10;
    public static double POWER_CAP = 0.9;

    private final String deviceName;
    private final String description;

    private DcMotorEx armMotor;
    private DcMotorEx liftMotor;
    private Servo containerServo;

    private PIDFController armController;
    private PIDFController liftController;

    private Position target = Position.Start;

    private final SizedStack<Integer> armPosition = new SizedStack<>(5);

    /**
     * Creates a new scoring arm with the given device name and description.
     * @param deviceName the name of the scoring arm.
     * @param description the description of the scoring arm.
     */
    public Arm(String deviceName, String description) {
        this.deviceName = deviceName;
        this.description = description;
    }

    /**
     * Initializes the scoring arm.
     * @param hardwareMap the map of all hardware on the robot.
     */
    @Override
    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        containerServo = hardwareMap.get(Servo.class, "containerFlip");

        initMotor(armMotor);
        initMotor(liftMotor);

        armController = new PIDFController(ARM_KP, ARM_KI, ARM_KD, ARM_KF);
        armController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        liftController = new PIDFController(LIFT_KP, LIFT_KI, LIFT_KD, LIFT_KF);
        liftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }

    /**
     * Initializes the arm motor.
     * @param motor the arm motor to be initialized.
     */
    private void initMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Returns an indication as to whether the arm and container are at their target positions.
     * @return <code>true</code> if both the arm and container are at their target positions;
     *         <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        int armPos = armMotor.getCurrentPosition();
        boolean atTarget = Math.abs(target.armPosition - armPos) <= TOLERABLE_ERROR;
        if (atTarget) {
            int liftPos = liftMotor.getCurrentPosition();
            atTarget = Math.abs(target.liftPosition - liftPos) <= TOLERABLE_ERROR;
        }
        if (atTarget) {
            double servoPos = containerServo.getPosition();
            atTarget = servoPos == target.servoPosition;
        }
        return atTarget;
    }

    /**
     * Sets the target position for the arm.
     * @param target the target position of the arm.
     */
    public void setTarget(Position target) {
        if (this.target != target) {
            resetPIDController();
            this.target = target;
        }
    }

    /**
     * Update sets the power for the arm motor based on the target position.
     */
    public void update() {
        // Check to see if power is being applied to the arm motor but the arm is not moving;
        // if so, then don't update the arm position. This is to avoid the "smack of doom" which
        // can occur if the arm motor's encoder isn't working properly.
        if (Math.abs(target.armPosition - armMotor.getCurrentPosition()) <= TOLERABLE_ERROR) {
            // Reached the target position, so the arm encoder is working
            armPosition.clear();
        } else {
            // After a full set of iterations to fill the arm position stack has occurred, check
            // to see if there is any change in the position between any two data points. If not,
            // then we have a possible "smack of doom" situation, so don't continue to supply
            // power to the arm's motor.
            armPosition.push(armMotor.getCurrentPosition());
            if (armPosition.size() == armPosition.capacity()) {
                boolean posChanged = false;
                int lastPos = armPosition.get(0);
                for (int i = 1; i < armPosition.size(); i++) {
                    if (lastPos != armPosition.get(i)) {
                        posChanged = true;
                        break;
                    }
                }
                if (!posChanged) {
                    return;
                }
            }
        }

        // No "smack of doom", so update the power for the arm and lift, and make sure the container
        // flip servo is set to the correct position.
        double power = calculate(armController);
        power = calculate(liftController);
        armMotor.setPower(power);
        liftMotor.setPower(power);
        containerServo.setPosition(target.servoPosition);
    }

    /**
     * Calculates the power to apply to the arm motor.
     * @return the power to apply to the motor.
     */
    private double calculate(PIDFController controller) {
        if (isAtTarget()) {
            return 0.0;
        }
        double power = controller.calculate();
        power = Range.clip(power, -POWER_CAP, POWER_CAP);

        return power;
    }

    /**
     * resetPIDController resets the PID control values.
     */
    private void resetPIDController() {
        armController.reset();
        armController.setPIDF(ARM_KP, ARM_KI, ARM_KD, ARM_KF);
        liftController.reset();
        liftController.setPIDF(LIFT_KP, LIFT_KI, LIFT_KD, LIFT_KF);
    }

    public int getCurrentArmPosition() {
        return armMotor.getCurrentPosition();
    }

    public int getCurrentLiftPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getCurrentContainerServoPosition() {
        return containerServo.getPosition();
    }

    public int getTargetArmPosition() {
        return target.armPosition;
    }

    public int getTargetLiftPosition() {
        return target.liftPosition;
    }

    public double getTargetContainerServoPosition() {
        return target.servoPosition;
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

    @Override
    public @NonNull String toString() {
        return string();
    }
}
