package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Autonomous mecanum drive.
 */
@Config
public class AutoMecanumDrive extends Drive {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    private Localizer localizer;
    private final MecanumDrive drive;

    private double kV;
    private double kA;
    private double kStatic;
    private double trackWidth;
    private double wheelBase;
    private double lateralMultiplier;

    private VoltageSensor batteryVoltageSensor;

    private TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectoryFollower follower;
    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.009, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(19, 0, 0);
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private final Telemetry telemetry;

    /**
     * Creates a new autonomous mecanum drive.
     */
    public AutoMecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.kV = DriveConstants.kV;
        this.kA = DriveConstants.kA;
        this.kStatic = DriveConstants.kStatic;
        this.trackWidth = DriveConstants.TRACK_WIDTH;
        this.wheelBase = DriveConstants.WHEEL_BASE;
        this.lateralMultiplier = DriveConstants.LATERAL_MULTIPLIER;

        drive = new MecanumDrive("drive", "Mecanum Drive", hardwareMap);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam Front");
        localizer = new DeadWheelLocalizer(hardwareMap, lastEncPositions, lastTrackingEncVels);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    /**
     * Returns the current localizer being used by the mecanum drive.
     * @return the current localizer being used by the mecanum drive.
     */
    @NonNull
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    /**
     * Sets the localizer to be used by the mecanum drive.
     * @param localizer the localizer to be used by the mecanum drive.
     */
    @Override
    public void setLocalizer(@NonNull Localizer localizer) {
        this.localizer = localizer;
    }

    /**
     * Unused when using three-wheel odometry.
     * @return <code>0.0</code>.
     */
    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    /**
     * Sets the motor powers by calculating the robot's drive power.
     * @param drivePower the robot's drive power.
     */
    @Override
    public void setDrivePower(@NonNull Pose2d drivePower) {
        List<Double> powers = MecanumKinematics.robotToWheelVelocities(
                drivePower,
                trackWidth,
                wheelBase,
                lateralMultiplier
        );
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    /**
     * Sets the motor powers based on the feed forward calculations using the robot's wheel
     * velocities and wheel accelerations.
     * @param driveSignal the robot's drive signal.
     */
    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocities(
                driveSignal.getVel(),
                trackWidth,
                wheelBase,
                lateralMultiplier
        );
        List<Double> accelerations = MecanumKinematics.robotToWheelAccelerations(
                driveSignal.getAccel(),
                trackWidth,
                wheelBase,
                lateralMultiplier
        );
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        setMotorPowers(powers.get(0), powers.get(1), powers.get(2), powers.get(3));
    }

    /**
     * Sets the power for the drive motors.
     * @param leftFront the power for the left front motor.
     * @param leftRear the power for the left rear motor.
     * @param rightRear the power for the right rear motor.
     * @param rightFront the power for the right front motor.
     */
    public void setMotorPowers(double leftFront, double leftRear, double rightRear, double rightFront) {
        drive.setMotorPowers(leftFront, leftRear, rightRear, rightFront);
    }

    /**
     * Updates the robot's trajectory and sets the robot's wheel velocities based on the pose
     * estimate and pose velocity.
     */
    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    /**
     * Returns a new trajectory builder based on the staring pose.
     * @param startPose the starting pose.
     * @return a trajectory builder based on the starting pose.
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Returns a new reversed trajectory builder based on the starting pose.
     * @param startPose the starting pose.
     * @param reversed <code>true</code> if the trajectory should be reversed;
     *                 <code>false</code> if it shouldn't be reversed.
     * @return a trajectory builder based on the starting pose.
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Returns a new trajectory builder given the starting pose and start tangent.
     * @param startPose the staring pose.
     * @param startHeading the start tangent.
     * @return a new trajectory builder given the staring pose and start tangent.
     */
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    /**
     * Returns a trajectory sequence builder based on the starting pose.
     * @param startPose the starting pose.
     * @return a trajectory sequence builder based on the starting pose.
     */
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    /**
     * Creates and asynchronously follows a trajectory to turn the robot by the specified angle.
     * @param angle the angle to turn the robot.
     */
    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    /**
     * Creates and follows a trajectory to turn the robot by the specified angle.
     * @param angle the specified angle to turn the robot.
     */
    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    /**
     * Asynchronously follows the specified trajectory.
     * @param trajectory the trajectory to follow.
     */
    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    /**
     * Follows the specified trajectory.
     * @param trajectory the specified trajectory.
     */
    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    /**
     * Asynchronously follows the trajectory sequence.
     * @param trajectorySequence the trajectory sequence to follow.
     */
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    /**
     * Follows the trajectory sequence.
     * @param trajectorySequence the trajectory sequence to follow.
     */
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    /**
     * Gets the last pose error from the following the trajectory sequence.
     * @return the last pose error from following the trajectory sequence.
     */
    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    /**
     * Continues to follow the robot's trajectory until the thread is interrupted or the
     * robot reaches it's destination.
     */
    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    /**
     * Returns indication as to whether the trajectory sequence is still being followed.
     * @return <code>true</code> if the trajectory sequence is no longer being followed;
     *         <code>false</code> if the trajectory sequence is still being followed.
     */
    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    /**
     * Sets the drive power to the weighted input drive power.
     * @param drivePower the drive power.
     */
    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    /**
     * Returns a trajectory velocity constraint.
     * @param maxVel the maximum velocity.
     * @param maxAngularVel the maximum angular velocity.
     * @param trackWidth the distance between two wheels on opposite sides of the robot.
     * @return the trajectory velocity constraint.
     */
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    /**
     * Returns an acceleration constraint.
     * @param maxAccel the maximum acceleration.
     * @return the acceleration constraint.
     */
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
