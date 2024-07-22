package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.CommandBaseEx;
import org.firstinspires.ftc.teamcode.command.CommandEx;
import org.firstinspires.ftc.teamcode.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.feedback.PIDControllerEx;

/**
 * A lift that uses two motors and slides to raise and lower an arm and two pixel collectors.
 */
@Config
public class Lift extends Subsystem {
    public static int INTAKE_POSITION = 0;
    public static int SCORE_LOW_POSITION = 350;
    public static int SCORE_MEDIUM_POSITION = 700;
    public static int SCORE_HIGH_POSITION = 1100;
    public static int HANG_START_POSITION = 1525;
    public static int HANG_END_POSITION = 900;
    public static int DRONE_LAUNCH_POSITION = 0;

    public static int ACCEPTABLE_ERROR = 10;

    public static double KP = 0.009;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KG = 0.1;
    public static double INTEGRAL_LIMIT = 1;
    public static double MAX_ACCELERATION = 3000;
    public static double MAX_VELOCITY = 8000;
    public static double MIN_POWER = 0.1;

    private final MotorEx leftMotor;
    private final MotorEx rightMotor;

    private final PIDControllerEx leftController;
    private final PIDControllerEx rightController;

    private MotionProfile motionProfile;

    private double targetPosition = INTAKE_POSITION;

    /**
     * Instantiates a new lift that controls the motors for raising and lower the lift.
     *
     * @param hardwareMap the hardware map that contains all the robot's hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        leftMotor = new MotorEx(hardwareMap, "leftLift");
        leftMotor.stopAndResetEncoder();
        leftMotor.setInverted(true);

        rightMotor = new MotorEx(hardwareMap, "rightMotor");
        rightMotor.stopAndResetEncoder();

        leftController = new PIDControllerEx(KP, KI, KD, KG);
        leftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        rightController = new PIDControllerEx(KP, KI, KD, KG);
        rightController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        // Build the motion profile to move the lift to the target position
        motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, leftMotor.getCurrentPosition(), INTAKE_POSITION);

        telemetry.addLine("[LIFT] initialized");
    }

    /**
     * Moves the lift to the specified position.
     *
     * @param position the position to which to set the lift.
     */
    public void setPosition(final int position) {
        if (targetPosition != position) {
            targetPosition = position;
            leftController.reset();
            rightController.reset();
            motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, leftMotor.getCurrentPosition(), position);

            telemetry.addData("[LIFT] set position", position);
        }
    }

    /**
     * Adjusts the lift position in order to reach the target position.
     */
    public void execute() {
        // Read the current position of each motor
        double leftPosition = leftMotor.getCurrentPosition();
        double rightPosition = rightMotor.getCurrentPosition();

        // Get the calculated target position from our motion profile
        double intermediateTarget = motionProfile.calculatePosition();
        telemetry.addData("[LIFT] intermediate target", intermediateTarget);

        // Get the error between the two positions
        double error = Math.abs(leftPosition - targetPosition);

        double leftPower, rightPower;
        if (targetPosition == INTAKE_POSITION && error < ACCEPTABLE_ERROR) {
            leftPower = 0;
            rightPower = 0;
        } else {
            // Calculate the power for each motor using the PID controllers
            leftPower = leftController.calculate(intermediateTarget, leftPosition);
            rightPower = rightController.calculate(intermediateTarget, rightPosition);

            // Cap the motor power at 1 and -1
            leftPower = Lift.modifyMotorPower(leftPower, MIN_POWER);
            rightPower = Lift.modifyMotorPower(rightPower, MIN_POWER);
        }

        setPower(leftPower, rightPower);
    }

    /**
     * Sets the power for the two lift motors.
     *
     * @param leftPower  the left motor for the lift.
     * @param rightPower the right motor for the lift.
     */
    public void setPower(double leftPower, double rightPower) {
        // Apply the power to each motor
        leftMotor.set(leftPower);
        rightMotor.set(rightPower);

        telemetry.addData("[LIFT] left power", leftPower);
        telemetry.addData("[LIFT] right power", rightPower);
        telemetry.addData("[LIFT] left position", leftMotor.getCurrentPosition());
        telemetry.addData("[LIFT] right position", rightMotor.getCurrentPosition());
    }

    /**
     * Returns an indication as to whether the lift is at the target position.
     *
     * @return <code>true</code> if the lift is at the target position;
     * <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        // Get the error between the two positions
        double error = Math.abs(leftMotor.getCurrentPosition() - targetPosition);

        final boolean finished;
        if (targetPosition == INTAKE_POSITION) {
            finished = error < ACCEPTABLE_ERROR;
        } else {
            finished = motionProfile.isAtEnd() && error < ACCEPTABLE_ERROR;
        }

        telemetry.addData("[LIFT] at target", finished);
        return finished;
    }

    /**
     * Returns an action to set the lift to the target position.
     *
     * @param position the position to which to set the lift.
     * @return the action to set the lift to the target position.
     */
    public CommandEx setToPosition(int position) {
        return new SetToPosition(this, position);
    }

    /**
     * Action used to set the lift to the target position.
     */
    private static class SetToPosition extends CommandBaseEx {
        private final Lift lift;
        private final int position;

        /**
         * Instantiates an action to move the lift to the target position.
         *
         * @param lift     the lift to be raised or lowered
         * @param position the position to which to move the lift
         */
        public SetToPosition(Lift lift, int position) {
            this.lift = lift;
            this.position = position;
        }

        /**
         * Initialize the action.
         */
        public void initialize() {
            lift.setPosition(position);
        }

        /**
         * Adjust the lift's position until it reaches the target position.
         */
        @Override
        public void execute() {
            lift.execute();
        }

        /**
         * Returns an indication as to whether the lift has reached the target.
         *
         * @return <code>true</code> if the lift is at the target position;
         * <code>false</code> if the lift is not at the target position.
         */
        @Override
        public boolean isFinished() {
            return super.isFinished();
        }
    }
}
