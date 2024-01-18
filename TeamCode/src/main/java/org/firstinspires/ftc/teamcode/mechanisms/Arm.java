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
    public static int ARM_INTAKE_POS = 0;
    public static int LIFT_INTAKE_POS = 0;
    public static double CONTAINER_INTAKE_POS = 0.65;
    public static int ARM_SCORE_LOW_POS = 2900;
    public static int LIFT_SCORE_LOW_POS = 0;
    public static double CONTAINER_SCORE_LOW_POS = 0.25;
    public static int ARM_SCORE_MIDDLE_POS = 2750;
    public static int LIFT_SCORE_MIDDLE_POS = 0;
    public static double CONTAINER_SCORE_MIDDLE_POS = 0.25;
    public static int ARM_SCORE_HIGH_POS = 2500;
    public static int LIFT_SCORE_HIGH_POS = 0;
    public static double CONTAINER_SCORE_HIGH_POS = 0.25;
    public static int ARM_START_POS = 0;
    public static int LIFT_START_POS = 0;
    public static double CONTAINER_START_POS = 0.65;
    public static int ARM_RELEASE_ROLLER_POS = 1200;
    public static int LIFT_RELEASE_ROLLER_POS = 0;
    public static double CONTAINER_RELEASE_ROLLER_POS = 0.65;
    public static int ARM_HANG_POS = 2000;
    public static int LIFT_HANG_POS = 0;
    public static double CONTAINER_HANG_POS = 0.65;

    public static int ARM_DRONE_LAUNCH_POS = 1250;
    public static int LIFT_DRONE_LAUNCH_POS = 0;
    public static double CONTAINER_DRONE_LAUNCH_POS = 0.65;

    public enum Position {
        Intake(ARM_INTAKE_POS, LIFT_INTAKE_POS, CONTAINER_INTAKE_POS),
        ScoreLow(ARM_SCORE_LOW_POS, LIFT_SCORE_LOW_POS, CONTAINER_SCORE_LOW_POS),
        ScoreMedium(ARM_SCORE_MIDDLE_POS, LIFT_SCORE_MIDDLE_POS, CONTAINER_SCORE_MIDDLE_POS),
        ScoreHigh(ARM_SCORE_HIGH_POS, LIFT_SCORE_HIGH_POS, CONTAINER_SCORE_HIGH_POS),
        Start(ARM_START_POS, LIFT_START_POS, CONTAINER_START_POS),
        ReleaseRoller(ARM_RELEASE_ROLLER_POS, LIFT_RELEASE_ROLLER_POS, CONTAINER_RELEASE_ROLLER_POS),
        Hang(ARM_HANG_POS, LIFT_HANG_POS, CONTAINER_HANG_POS),
        LaunchDrone(ARM_DRONE_LAUNCH_POS, LIFT_DRONE_LAUNCH_POS, CONTAINER_DRONE_LAUNCH_POS);

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

    // PID control constants. TODO: change to private final once tuned
    public static double ARM_KP = 0.005;
    public static double ARM_KI = 0.0;
    public static double ARM_KD = 0.0;
    public static double LIFT_KP = 0.0;
    public static double LIFT_KI = 0.0;
    public static double LIFT_KD = 0.0;
    public static double INTEGRAL_LIMIT = 1;

    public static int TOLERABLE_ERROR = 20;
    public static double POWER_CAP = 0.9;
    public static double MIN_POWER = 0.1;

    private final String deviceName;
    private final String description;

    private DcMotorEx armMotor;
    // private DcMotorEx liftMotor;
    private Servo containerServo;

    private PIDController armController;
    // private PIDFController liftController;

    private Position target = Position.Start;

    private final SizedStack<Integer> armPosition = new SizedStack<>(100);

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
        // initMotor(liftMotor);

        armController = new PIDController(new PIDCoefficients(ARM_KP, ARM_KI, ARM_KD));
        armController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        // liftController = new PIDFController(LIFT_KP, LIFT_KI, LIFT_KD, LIFT_KF);
        // liftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
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
        // if (atTarget) {
            // int liftPos = liftMotor.getCurrentPosition();
            // atTarget = Math.abs(target.liftPosition - liftPos) <= TOLERABLE_ERROR;
        // }
        if (atTarget) {
            double servoPos = getCurrentContainerServoPosition();
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

    public void disableRunToPosition() {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            armPosition.clear();
            return;
        }

        // Check for the "smack of doom" scenario
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
                telemetry.addLine("[ARM] Smack of Doom!!!");
                return;
            }
        }

        // No "smack of doom", so update the power for the arm and lift, and make sure the container
        // flip servo is set to the correct position.
        double power = calculate(armController, target.armPosition, armMotor.getCurrentPosition());
        telemetry.addData("[ARM] Power", power);
        armMotor.setPower(power);
        // power = calculate(liftController);
        // liftMotor.setPower(power);
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

    /**
     * resetPIDController resets the PID control values.
     */
    private void resetPIDController() {
        armController = new PIDController(new PIDCoefficients(ARM_KP, ARM_KI, ARM_KD));
        armController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        // liftController.reset();
        // liftController.setPIDF(LIFT_KP, LIFT_KI, LIFT_KD, LIFT_KF);
    }

    public int getCurrentArmPosition() {
        return armMotor.getCurrentPosition();
    }

    // public int getCurrentLiftPosition() {
    //    return liftMotor.getCurrentPosition();
    // }

    public double getCurrentContainerServoPosition() {
        double servoPos = containerServo.getPosition();
        long round = Math.round(servoPos * 100);
        servoPos = ((float) round) / 100.0;
        return servoPos;
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
