package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.CommandBaseEx;
import org.firstinspires.ftc.teamcode.command.CommandEx;
import org.firstinspires.ftc.teamcode.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.feedback.PIDController;

/**
 * The arm mechanism that moves the pixel collector from the intake position to the scoring
 * position and vice-versa.
 */
@Config
public class Arm extends Subsystem {
    public static int INTAKE_POSITION = 0;
    public static int SCORE_POSITION = 2700;

    public static double KP = 0.003;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double MAX_ACCELERATION = 3000;
    public static double MAX_VELOCITY = 5000;
    public static double MIN_POWER = 0.16;
    public static double ACCEPTABLE_ERROR = 10;

    private final MotorEx motor;
    private final PIDController pidController;
    private double targetPosition = INTAKE_POSITION;
    private MotionProfile motionProfile;

    /**
     * Instantiates a new arm for the robot.
     *
     * @param hardwareMap the mapping of the hardware on the robot.
     * @param telemetry   the telemetry used to output information to the user.
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        motor = new MotorEx(hardwareMap, "armMotor");
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidController = new PIDController(KP, KI, KD);
        motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, 0, INTAKE_POSITION);

        telemetry.addLine("[ARM] initialized");
    }

    /**
     * Sets the target position for the arm.
     *
     * @param position the target position for the arm.
     */
    public void setPosition(final int position) {
        if (targetPosition != position) {
            targetPosition = position;
            pidController.reset();
            motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, motor.getCurrentPosition(), position);

            telemetry.addData("[ARM] set position", position);
        }
    }

    /**
     * Adjusts the arm position to reach the target position.
     */
    public void execute() {
        // Read the current position of each motor
        double position = motor.getCurrentPosition();

        // Get the calculated target position from our motion profile
        double intermediateTarget = motionProfile.calculatePosition();
        telemetry.addData("[ARM] intermediate target", intermediateTarget);

        // Get the error between the two positions
        double error = Math.abs(position - targetPosition);

        final double power;
        if (targetPosition == INTAKE_POSITION && error < ACCEPTABLE_ERROR) {
            power = 0;
        } else {
            // Calculate the power for each motor using the PID controllers
            power = pidController.calculate(intermediateTarget, position);
        }

        setPower(power);
    }

    /**
     * Sets the power for the two lift motors.
     *
     * @param power the power for the arm motor.
     */
    public void setPower(double power) {
        // Apply the power to the arm motor
        motor.set(power);

        telemetry.addData("[ARM] target", targetPosition);
        telemetry.addData("[ARM] power", power);
        telemetry.addData("[ARM] position", motor.getCurrentPosition());
    }

    /**
     * Returns an indication as to whether the arm is at the target position.
     *
     * @return <code>true</code> if the arm is at the target position;
     * <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        // Get the error between the two positions
        double error = Math.abs(motor.getCurrentPosition() - targetPosition);

        final boolean finished;
        if (targetPosition == INTAKE_POSITION) {
            finished = error < ACCEPTABLE_ERROR;
        } else {
            finished = motionProfile.isAtEnd() && error > ACCEPTABLE_ERROR;
        }

        telemetry.addData("[ARM] at target", finished);
        return finished;
    }

    /**
     * Gets an action to raise the arm to the scoring position.
     *
     * @return an action to raise the arm to the scoring position.
     */
    public CommandEx raiseArm() {
        return new SetToPosition(this, SCORE_POSITION);
    }

    /**
     * Gets an action to lower the arm to the intake position.
     *
     * @return an action to lower the arm to the intake position.
     */
    public CommandEx lowerArm() {
        return new SetToPosition(this, INTAKE_POSITION);
    }

    /**
     * Action used to set the arm to the target position.
     */
    private static class SetToPosition extends CommandBaseEx {
        private final Arm arm;
        private final int position;
        private boolean initialized = false;

        /**
         * Instantiates an action to move the arm to the target position.
         *
         * @param arm      the arm to be raised or lowered
         * @param position the position to which to move the lift
         */
        public SetToPosition(Arm arm, int position) {
            this.arm = arm;
            this.position = position;

            addRequirements(arm);
        }

        /**
         * Initialize the action.
         */
        @Override
        public void initialize() {
            arm.setPosition(position);
        }

        @Override
        public void execute() {
            arm.execute();
        }

        /**
         * Returns an indication as to whether the arm has reached the target position.
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
