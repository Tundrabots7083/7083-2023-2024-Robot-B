package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.feedback.PIDCoefficients;
import org.firstinspires.ftc.teamcode.feedback.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tests.Test;

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
        Start(0, 0, 0),
        Intake(0, 0, 0.61),
        ScoreLow(2900, 0, 0.25),
        ScoreMedium(2750, 0, 0.25),
        ScoreHigh(2500, 0, 0.25),
        Hang(2000, 0, 0.65),
        LaunchDrone(1455, 0, 1.0);

        public final int armPosition;
        public final int liftPosition;
        public  final double servoPosition;

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

    // PID control constants. TODO: change to private final once tuned
    public static double ARM_KP = 0.0053;
    public static double ARM_KI = 0.0;
    public static double ARM_KD = 0.0;
    public static double INTEGRAL_LIMIT = 1;

    public static int TOLERABLE_ERROR = 20;
    public static double POWER_CAP = 0.9;
    public static double MIN_POWER = 0.1;

    private final String deviceName;
    private final String description;

    private final DcMotorEx armMotor;
    // private DcMotorEx liftMotor;
    private final Servo containerServo;

    private PIDController armController;
    // private PIDFController liftController;

    private Position target = Position.Start;

    /**
     * Creates a new scoring arm with the given device name and description.
     * @param deviceName the name of the scoring arm.
     * @param description the description of the scoring arm.
     */
    public Arm(String deviceName, String description, HardwareMap hardwareMap) {
        this.deviceName = deviceName;
        this.description = description;

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        // liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        containerServo = hardwareMap.get(Servo.class, "containerFlip");

        initMotor(armMotor);

        armController = new PIDController(new PIDCoefficients(ARM_KP, ARM_KI, ARM_KD));
        armController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }

    /**
     * Initializes the arm motor.
     * @param motor the arm motor to be initialized.
     */
    private void initMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
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
            double servoPos = getCurrentContainerServoPosition();
            atTarget = (servoPos == target.servoPosition);
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

    public void setArmPower(double power) {
        power = Range.clip(power, -1, 1);
        armMotor.setPower(power);
    }

    /**
     * Update sets the power for the arm motor based on the target position.
     */
    public void update() {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        // Short-circuit the processing if the arm is at it's target
        if (isAtTarget()) {
            return;
        }

        // No "smack of doom", so update the power for the arm and lift, and make sure the container
        // flip servo is set to the correct position.
        double power = calculate(armController, target.armPosition, armMotor.getCurrentPosition());
        telemetry.addData("[ARM] Power", power);
        armMotor.setPower(power);
        containerServo.setPosition(target.servoPosition);
    }

    /**
     * Calculates the power to apply to the arm motor.
     * @return the power to apply to the motor.
     */
    private double calculate(PIDController controller, double target, double current) {
        double power = -controller.calculate(target, current);
        power = Range.clip(power, -POWER_CAP, POWER_CAP);
        if (power > 0.0 && power < MIN_POWER) {
            power = MIN_POWER;
        } else if (power < 0.0 && power > -MIN_POWER) {
            power = -MIN_POWER;
        }

        return power;
    }

    public void start() {
        target = Position.Intake;
        containerServo.setPosition(target.servoPosition);
        update();
    }

    /**
     * resetPIDController resets the PID control values.
     */
    private void resetPIDController() {
        armController = new PIDController(new PIDCoefficients(ARM_KP, ARM_KI, ARM_KD));
        armController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    }

    public int getCurrentArmPosition() {
        return armMotor.getCurrentPosition();
    }

    public double getCurrentContainerServoPosition() {
        double servoPos = containerServo.getPosition();
        long round = Math.round(servoPos * 100);
        servoPos = ((float) round) / 100.0;
        return servoPos;
    }

    public int getTargetArmPosition() {
        return target.armPosition;
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
