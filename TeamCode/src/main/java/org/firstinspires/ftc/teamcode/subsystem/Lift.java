package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.feedback.PIDController;
import org.firstinspires.ftc.teamcode.feedback.PIDControllerEx;

/**
 * Lift subsystem responsible for lifting and lowering the pixel collector.
 */
@Config
public class Lift extends SubsystemBaseEx {
    public static double KP = 0.009;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KG = 0.1;
    public static double INTEGRAL_LIMIT = 1;

    public static double MAX_ACCELERATION = 3000;
    public static double MAX_VELOCITY = 8000;
    public static double MIN_POWER = 0.1;

    public static double ACCEPTABLE_ERROR = 10;

    public static int INTAKE_POSITION = 0;
    public static int SCORE_LOW_POSITION = -350;
    public static int SCORE_MEDIUM_POSITION = -700;
    public static int SCORE_HIGH_POSITION = -1100;
    public static int SCORE_AUTONOMOUS_BACKSTAGE_POSITION = -215;
    public static int SCORE_AUTONOMOUS_FRONTSTAGE_POSITION = -350;
    public static int HANG_START_POSITION = -1525;
    public static int HANG_END_POSITION = -900;
    public static int DRONE_LAUNCH_POSITION = 0;

    private final Telemetry telemetry;
    private final Motor leftMotor;
    private final Motor rightMotor;
    private final PIDController leftController;
    private final PIDController rightController;
    private MotionProfile liftProfile;
    private Position targetPosition = Position.INTAKE;

    /**
     * Creates a new Lift hardware subsystem that raises and lowers the pixel collector
     *
     * @param hardwareMap the hardware map that contains the hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftMotor = new Motor(hardwareMap, "leftLift");
        leftMotor.stopAndResetEncoder();

        rightMotor = new Motor(hardwareMap, "rightLift");
        rightMotor.stopAndResetEncoder();

        leftController = new PIDControllerEx(KP, KI, KD, -KG);
        rightController = new PIDControllerEx(KP, KI, KD, -KG);

        leftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        rightController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        liftProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, leftMotor.getCurrentPosition(), INTAKE_POSITION);
    }

    /**
     * Sets the target position for the lift.
     *
     * @param position the target position for the lift
     */
    public void setTarget(Position position) {
        if (targetPosition != position) {
            targetPosition = position;
            int pos = position.getValue();
            telemetry.addData("[LIFT] Set Position", targetPosition);

            // Build the motion profile to move the lift to the target position
            liftProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, leftMotor.getCurrentPosition(), position.getValue());

            // Reset the PID controllers
            leftController.reset();
            rightController.reset();
        }
    }

    /**
     * Returns indication as to whether the lift is at the target position.
     *
     * @return <code>true</code> if the lift is at the target position;
     * <code>false</code> if the lift is not.
     */
    public boolean isAtTarget() {
        return liftProfile.isAtEnd() &&
                Math.abs(leftMotor.getCurrentPosition()) < ACCEPTABLE_ERROR &&
                Math.abs(rightMotor.getCurrentPosition()) < ACCEPTABLE_ERROR;
    }

    /**
     * Adjusts the lift's position to match the target position
     */
    @Override
    public void execute() {
        // Read the current position of each motor
        int leftPosition = leftMotor.getCurrentPosition();
        int rightPosition = rightMotor.getCurrentPosition();

        // Get the target position from our motion profile
        double targetPos = liftProfile.calculatePosition();

        // Get the error between the two positions
        double error = Math.abs(leftPosition - targetPos);

        double leftPower, rightPower;
        if (targetPosition == Position.INTAKE && error < ACCEPTABLE_ERROR) {
            leftPower = 0;
            rightPower = 0;
        } else {
            // Calculate the power for each motor using the PID controllers
            leftPower = leftController.calculate(targetPos, leftPosition);
            rightPower = rightController.calculate(targetPos, rightPosition);

            // Cap the motor power at 1 and -1
            leftPower = modifyMotorPower(leftPower, MIN_POWER);
            rightPower = modifyMotorPower(rightPower, MIN_POWER);
        }

        // Apply the power to each motor
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);

        telemetry.addData("[LIFT] target", targetPosition);
        telemetry.addData("[LIFT] target position", targetPos);
        telemetry.addData("[LIFT] left position", leftPosition);
        telemetry.addData("[LIFT] right position", rightPosition);
        telemetry.addData("[LIFT] left power", leftPower);
        telemetry.addData("[LIFT] right power", rightPower);
    }

    /**
     * Manually set the power for the lift motors.
     *
     * @param power the power to set both lift motors to
     */
    public void overrideLiftPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);

        telemetry.addData("[LIFT] left position", leftMotor.getCurrentPosition());
        telemetry.addData("[LIFT] right position", rightMotor.getCurrentPosition());
        telemetry.addData("[LIFT] left power", leftMotor.get());
        telemetry.addData("[LIFT] right power", rightMotor.get());
    }

    /**
     * Position to which to move the lift
     */
    public enum Position {
        INTAKE,
        AUTONOMOUS_BACKSTAGE,
        AUTONOMOUS_FRONTSTAGE,
        SCORE_LOW,
        SCORE_MEDIUM,
        SCORE_HIGH,
        HANG_START,
        HANG_END,
        LAUNCH_DRONE;

        /**
         * Gets the numerical value for the position. This is done via a switch statement so that
         * FTC Dashboard can edit the values; otherwise, it would make sense to include them within
         * the enum itself.
         *
         * @return the numeric value corresponding to the lift position
         */
        public int getValue() {
            final int pos;
            switch (this) {
                case INTAKE:
                    pos = INTAKE_POSITION;
                    break;
                case AUTONOMOUS_BACKSTAGE:
                    pos = SCORE_AUTONOMOUS_BACKSTAGE_POSITION;
                    break;
                case AUTONOMOUS_FRONTSTAGE:
                    pos = SCORE_AUTONOMOUS_FRONTSTAGE_POSITION;
                    break;
                case SCORE_LOW:
                    pos = SCORE_LOW_POSITION;
                    break;
                case SCORE_MEDIUM:
                    pos = SCORE_MEDIUM_POSITION;
                    break;
                case SCORE_HIGH:
                    pos = SCORE_HIGH_POSITION;
                    break;
                case HANG_START:
                    pos = HANG_START_POSITION;
                    break;
                case HANG_END:
                    pos = HANG_END_POSITION;
                    break;
                case LAUNCH_DRONE:
                    pos = DRONE_LAUNCH_POSITION;
                    break;
                default:
                    pos = 0;
            }

            return pos;
        }
    }
}
