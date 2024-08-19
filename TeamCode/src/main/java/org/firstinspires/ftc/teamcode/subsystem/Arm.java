package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.feedback.PIDController;

/**
 * Robot arm. This is connected to the pixel collectors and is used to swing the collectors from
 * the intake position to the scoring position and vice-versa.
 */
@Config
public class Arm extends SubsystemBaseEx {
    public static double INTEGRAL_LIMIT = 1;
    public static double KP = 0.003;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double MAX_ACCELERATION = 3000;
    public static double MAX_VELOCITY = 5000;
    public static double MIN_POWER = 0.16;
    public static int ACCEPTABLE_ERROR = 10;
    public static int INTAKE_POSITION = 0;
    public static int SCORING_POSITION = -2700;

    private final Telemetry telemetry;
    private final Motor motor;
    private final PIDController pidController;
    private MotionProfile motionProfile;

    private Position targetPosition = Position.INTAKE;

    /**
     * Creates a new Lift hardware mechanism that controls both the two lift motors and the arm
     * motor.
     *
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = new Motor(hardwareMap, "armMotor");
        motor.stopAndResetEncoder();

        pidController = new PIDController(KP, KI, KD);
        pidController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, motor.getCurrentPosition(), INTAKE_POSITION);
    }

    /**
     * Sets the target position for the arm.
     *
     * @param position the target position for the arm
     */
    public void setTarget(Position position) {
        if (targetPosition != position) {

            targetPosition = position;
            telemetry.addData("[ARM] Set Position", position.getValue());

            // Build the motion profile to move the arm to the target position
            motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, motor.getCurrentPosition(), position.getValue());

            // Reset the PID controller
            pidController.reset();
        }
    }

    /**
     * Returns indication as to whether the arm is at the target position.
     *
     * @return <code>true</code> if the arm is at the target position;
     * <code>false</code> if the arm is not.
     */
    public boolean isAtTarget() {
        return motionProfile.isAtEnd() && Math.abs(motor.getCurrentPosition()) < ACCEPTABLE_ERROR;
    }

    /**
     * Adjust the arm's power to continue to move it to the target location
     */
    @Override
    public void execute() {
        // Read the current position of the arm motor
        int armPosition = motor.getCurrentPosition();

        // Get the target position from our motion profile
        double targetPosition = motionProfile.calculatePosition();

        // Get the error between the two positions
        double error = Math.abs(armPosition - targetPosition);

        double power;
        if (this.targetPosition == Position.INTAKE && error < ACCEPTABLE_ERROR) {
            power = 0;
        } else {
            // Calculate the power for the arm motor using the PID controller
            power = pidController.calculate(targetPosition, armPosition);

            // Cap the motor power at 1 and -1
            power = modifyMotorPower(power, MIN_POWER);
        }

        // Apply the power to the arm motor
        motor.set(power);

        telemetry.addData("[ARM] target", targetPosition);
        telemetry.addData("[ARM] power", power);
        telemetry.addData("[ARM] position", armPosition);
    }

    /**
     * Manual override for the arm power
     *
     * @param power the power to which to set the arm
     */
    public void overrideArmPower(double power) {
        // Cap the motor power at 1 and -1
        power = modifyMotorPower(power, MIN_POWER);

        // Set the motor's power
        motor.set(power);
        telemetry.addData("[ARM] power", power);
        telemetry.addData("[ARM] position", motor.getCurrentPosition());
    }

    /**
     * Arm positions
     */
    public enum Position {
        INTAKE,
        SCORING;

        /**
         * Returns the integer value related to the position. Ideally, this would be included as
         * a parameter for the Position enum, but doing so won't allow the values to be edited
         * in FTC Dashboard.
         *
         * @return the integer value related to the position
         */
        public int getValue() {
            return this == INTAKE ? INTAKE_POSITION : SCORING_POSITION;
        }
    }
}
