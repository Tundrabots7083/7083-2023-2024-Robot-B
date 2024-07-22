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

    public void setTarget(Position position) {
        if (targetPosition != position) {
            switch (position) {
                case INTAKE:
                    lift.setTarget(Lift.Position.INTAKE);
                    arm.setTarget(Arm.Position.INTAKE);
                    break;
                case AUTONOMOUS_BACKSTAGE:
                    lift.setTarget(Lift.Position.AUTONOMOUS_BACKSTAGE);
                    arm.setTarget(Arm.Position.SCORING);
                    break;
                case AUTONOMOUS_FRONTSTAGE:
                    lift.setTarget(Lift.Position.AUTONOMOUS_FRONTSTAGE);
                    arm.setTarget(Arm.Position.SCORING);
                    break;
                case SCORE_LOW:
                    lift.setTarget(Lift.Position.SCORE_LOW);
                    arm.setTarget(Arm.Position.SCORING);
                    break;
                case SCORE_MEDIUM:
                    lift.setTarget(Lift.Position.SCORE_MEDIUM);
                    arm.setTarget(Arm.Position.SCORING);
                    break;
                case SCORE_HIGH:
                    lift.setTarget(Lift.Position.SCORE_HIGH);
                    arm.setTarget(Arm.Position.SCORING);
                    break;
                case HANG_START:
                    lift.setTarget(Lift.Position.HANG_START);
                    arm.setTarget(Arm.Position.INTAKE);
                    break;
                case HANG_END:
                    lift.setTarget(Lift.Position.HANG_END);
                    arm.setTarget(Arm.Position.INTAKE);
                    break;
                case LAUNCH_DRONE:
                    lift.setTarget(Lift.Position.LAUNCH_DRONE);
                    arm.setTarget(Arm.Position.INTAKE);
                    break;
            }

            targetPosition = position;
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
        INTAKE,
        AUTONOMOUS_BACKSTAGE,
        AUTONOMOUS_FRONTSTAGE,
        SCORE_LOW,
        SCORE_MEDIUM,
        SCORE_HIGH,
        HANG_START,
        HANG_END,
        LAUNCH_DRONE
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
