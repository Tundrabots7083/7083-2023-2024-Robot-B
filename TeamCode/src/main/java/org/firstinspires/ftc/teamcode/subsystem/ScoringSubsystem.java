package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A collection of components used for scoring. For CenterStage, this includes the lift and arm.
 */
public class ScoringSubsystem extends Subsystem {
    public final Lift lift;
    public final Arm arm;

    /**
     * Creates a new subsystem that uses the supplied telemetry for displaying output.
     *
     * @param hardwareMap the mapping of hardware for the robot
     * @param telemetry the telemetry to use for output.
     */
    public ScoringSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        this.lift = new Lift(hardwareMap, telemetry);
        this.arm = new Arm(hardwareMap, telemetry);
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
            case LAUNCH_DRONE:
                lift.setPosition(Lift.DRONE_LAUNCH_POSITION);
                arm.setPosition(Arm.INTAKE_POSITION);
                break;
            case INTAKE:
            case HANG_END:
            default:
                lift.setPosition(Lift.INTAKE_POSITION);
                arm.setPosition(Arm.INTAKE_POSITION);
        }
    }

    /**
     * Sets the lift and arm positions to the targets.
     *
     * @param position the position to which to move the lift and arm.
     * @return the action to move the lift and arm to the target positions.
     */
    public Action setTo(Position position) {
        final Action action;
        switch (position) {
            case SCORE_LOW:
            case AUTONOMOUS_FRONTSTAGE:
                action = new ParallelAction(
                        lift.setToPosition(Lift.SCORE_LOW_POSITION),
                        arm.raiseArm()
                );
                break;
            case SCORE_MEDIUM:
            case AUTONOMOUS_BACKSTAGE:
                action = new ParallelAction(
                        lift.setToPosition(Lift.SCORE_MEDIUM_POSITION),
                        arm.raiseArm()
                );
                break;
            case SCORE_HIGH:
                action = new ParallelAction(
                        lift.setToPosition(Lift.SCORE_HIGH_POSITION),
                        arm.raiseArm()
                );
                break;
            case HANG_START:
                action = new ParallelAction(
                        lift.setToPosition(Lift.HANG_START_POSITION),
                        arm.lowerArm()
                );
                break;
            case LAUNCH_DRONE:
                action = new ParallelAction(
                        lift.setToPosition(Lift.DRONE_LAUNCH_POSITION),
                        arm.lowerArm()
                );
                break;
            case INTAKE:
            case HANG_END:
            default:
                action = new ParallelAction(
                        lift.setToPosition(Lift.INTAKE_POSITION),
                        arm.lowerArm()
                );
        }
        return action;
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
