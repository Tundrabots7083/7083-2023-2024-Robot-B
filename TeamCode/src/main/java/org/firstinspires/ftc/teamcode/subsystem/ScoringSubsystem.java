package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The scoring subsystem for the robot, consisting of a lift and an arm.
 */
public class ScoringSubsystem extends SubsystemBaseEx {
    private final Telemetry telemetry;
    private final Arm arm;
    private final Lift lift;
    private Position targetPosition = Position.INTAKE;

    /**
     * Creates a new Lift hardware mechanism that controls both the two lift motors and the arm
     * motor.
     *
     * @param lift the lift used to raise and lower the pixel collectors
     * @param  arm the arm used to swing the pixel collectors between the scoring position and
     *             the intake position
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public ScoringSubsystem(Lift lift, Arm arm, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.arm = arm;
        this.lift = lift;
    }

    /**
     * Sets the target position for the lift subsystem.
     *
     * @param position the target position
     */
    public void setTarget(Position position) {
        if (targetPosition != position) {
            lift.setTarget(position.liftPosition);
            arm.setTarget(position.armPosition);
        }
    }

    /**
     * Returns indication as to whether both the arm and lift are at the target positions.
     *
     * @return <code>true</code> if the arm and lift are at the target position;
     * <code>false</code> if one or both is not.
     */
    public boolean isAtTarget() {
        return arm.isAtTarget() && lift.isAtTarget();
    }

    /**
     * Updates the position of the arm and lift to match the target position.
     */
    @Override
    public void execute() {
        arm.execute();
        lift.execute();
    }

    /**
     * Position to which to move the scoring subsystem
     */
    public enum Position {
        INTAKE(Arm.Position.INTAKE, Lift.Position.INTAKE),
        AUTONOMOUS_BACKSTAGE(Arm.Position.SCORING, Lift.Position.AUTONOMOUS_BACKSTAGE),
        AUTONOMOUS_FRONTSTAGE(Arm.Position.SCORING, Lift.Position.AUTONOMOUS_FRONTSTAGE),
        SCORE_LOW(Arm.Position.SCORING, Lift.Position.SCORE_LOW),
        SCORE_MEDIUM(Arm.Position.SCORING, Lift.Position.SCORE_MEDIUM),
        SCORE_HIGH(Arm.Position.SCORING, Lift.Position.SCORE_HIGH),
        HANG_START(Arm.Position.INTAKE, Lift.Position.HANG_START),
        HANG_END(Arm.Position.INTAKE, Lift.Position.HANG_END),
        LAUNCH_DRONE(Arm.Position.INTAKE, Lift.Position.LAUNCH_DRONE);

        private final Arm.Position armPosition;
        private final Lift.Position liftPosition;

        /**
         * Instantiate a Position enum with the given arm and lift positions.
         *
         * @param armPosition   the corresponding arm position
         * @param liftPosition the corresponding lift position
         */
        private Position(Arm.Position armPosition, Lift.Position liftPosition) {
            this.armPosition = armPosition;
            this.liftPosition = liftPosition;
        }

        /**
         * Get the arm position that is used for this scoring subsystem position.
         *
         * @return the arm position
         */
        public Arm.Position getArmPosition() {
            return armPosition;
        }

        /**
         * Get the lift position that is used for this scoring subsystem position.
         *
         * @return the lift position
         */
        public Lift.Position getLiftPosition() {
            return liftPosition;
        }
    }

    /**
     * Gets an action to set the scoring subsystem to the target position.
     *
     * @param position the position to which to set the scoring subsystem.
     * @return the action to set the scoring subsystem to the target position.
     */
    public Action setTo(Position position) {
        return null;
    }

    /**
     * Action used to set the scoring subsystem to the target position.
     */
    private static class SetPosition implements Action {
        private final ScoringSubsystem scoringSubsystem;
        private final Position targetPosition;
        private boolean initialized = false;

        public SetPosition(ScoringSubsystem scoringSubsystem, Position position) {
            this.scoringSubsystem = scoringSubsystem;
            this.targetPosition = position;
        }

        private void initialize() {
            scoringSubsystem.setTarget(targetPosition);
        }

        /**
         * Updates the position of the scoring subsystem to match the target position.
         *
         * @param telemetryPacket telemetry used to output data.
         * @return <code>false</code> if the scoring subsystem is at the target position;
         * <code>true</code> if it is not, and therefore needs to continue to run.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
                initialized = true;
            }

            scoringSubsystem.execute();

            return !scoringSubsystem.isAtTarget();
        }
    }
}
