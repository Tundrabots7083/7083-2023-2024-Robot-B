package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.CommandEx;
import org.firstinspires.ftc.teamcode.command.ParallelCommandGroupEx;

/**
 * A collection of components used for scoring. For CenterStage, this includes the lift and arm.
 */
public class ScoringSubsystem extends Subsystem {
    private final Arm arm;
    private final Lift lift;

    /**
     * Creates a new subsystem that uses the supplied telemetry for displaying output.
     *
     * @param arm       The robot arm, used for moving the pixel collector from the intake position
     *                  to the scoring position and vice-versa.
     * @param lift      The robot lift, used for elevating the pixel collector, as well as to hang
     *                  robot.
     * @param telemetry the telemetry to use for output.
     */
    public ScoringSubsystem(Arm arm, Lift lift, Telemetry telemetry) {
        super(telemetry);

        this.arm = arm;
        this.lift = lift;
    }

    /**
     * Updates the scoring subsystem components.
     */
    public void execute() {
        lift.execute();
        arm.execute();
    }

    /**
     * Returns an indication as to whether the lift and arm are at the target position.
     *
     * @return <code>true</code> if both the lift and arm are at the target position;
     * <code>false</code> if either is not.
     */
    public boolean isAtTarget() {
        return lift.isAtTarget() && arm.isAtTarget();
    }

    /**
     * Sets the position of the scoring system.
     *
     * @param position the position to set the scoring position to
     */
    public void setPosition(Position position) {
        switch (position) {
            case INTAKE:
                lift.setPosition(Lift.INTAKE_POSITION);
                arm.setPosition(Arm.INTAKE_POSITION);
            case SCORE_LOW:
            case AUTONOMOUS_FRONTSTAGE:
                lift.setPosition(Lift.SCORE_LOW_POSITION);
                arm.setPosition(Arm.SCORE_POSITION);
                break;
            case SCORE_MEDIUM:
            case AUTONOMOUS_BACKSTAGE:
                lift.setPosition(Lift.SCORE_MEDIUM_POSITION);
                arm.setPosition(Arm.SCORE_POSITION);
                break;
            case SCORE_HIGH:
                lift.setPosition(Lift.SCORE_HIGH_POSITION);
                arm.setPosition(Arm.SCORE_POSITION);
                break;
            case HANG_START:
                lift.setPosition(Lift.HANG_START_POSITION);
                arm.setPosition(Arm.INTAKE_POSITION);
                break;
            case HANG_END:
                lift.setPosition(Lift.HANG_END_POSITION);
                arm.setPosition(Arm.INTAKE_POSITION);
                break;
            case LAUNCH_DRONE:
                lift.setPosition(Lift.DRONE_LAUNCH_POSITION);
                arm.setPosition(Arm.INTAKE_POSITION);
                break;
        }
    }

    /**
     * Sets the lift and arm positions to the targets.
     *
     * @param position the position to which to move the lift and arm.
     * @return the action to move the lift and arm to the target positions.
     */
    public CommandEx setToPosition(Position position) {
        ParallelCommandGroupEx command = new ParallelCommandGroupEx();
        switch (position) {
            case INTAKE:
                command.addCommands(
                        lift.setToPosition(Lift.INTAKE_POSITION),
                        arm.lowerArm()
                );
            case SCORE_LOW:
            case AUTONOMOUS_FRONTSTAGE:
                command.addCommands(
                        lift.setToPosition(Lift.SCORE_LOW_POSITION),
                        arm.raiseArm()
                );
                break;
            case SCORE_MEDIUM:
            case AUTONOMOUS_BACKSTAGE:
                command.addCommands(
                        lift.setToPosition(Lift.SCORE_MEDIUM_POSITION),
                        arm.raiseArm()
                );
                break;
            case SCORE_HIGH:
                command.addCommands(
                        lift.setToPosition(Lift.SCORE_HIGH_POSITION),
                        arm.raiseArm()
                );
                break;
            case HANG_START:
                command.addCommands(
                        lift.setToPosition(Lift.HANG_START_POSITION),
                        arm.lowerArm()
                );
                break;
            case HANG_END:
                command.addCommands(
                        lift.setToPosition(Lift.HANG_END_POSITION),
                        arm.lowerArm()
                );
                break;
            case LAUNCH_DRONE:
                command.addCommands(
                        lift.setToPosition(Lift.DRONE_LAUNCH_POSITION),
                        arm.lowerArm()
                );
                break;
        }

        command.addRequirements(this, arm, lift);
        return command;
    }

    /**
     * Position to which to move the lift and arm\
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
        LAUNCH_DRONE,
    }
}
