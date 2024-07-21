package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class SetScoringSubsystemPositionCommand extends CommandBaseEx {
    private final ParallelCommandGroup moveCommand;

    public SetScoringSubsystemPositionCommand(Lift lift, Arm arm, Position position) {
        final long armPosition;
        final long liftPosition;

        switch (position) {
            case INTAKE:
                armPosition = Arm.INTAKE_POSITION;
                liftPosition = Lift.INTAKE_POSITION;
                break;
            case SCORE_LOW:
                armPosition = Arm.SCORE_POSITION;
                liftPosition = Lift.SCORE_LOW_POSITION;
                break;
            case SCORE_MEDIUM:
                armPosition = Arm.SCORE_POSITION;
                liftPosition = Lift.SCORE_MEDIUM_POSITION;
                break;
            case SCORE_HIGH:
                liftPosition = Lift.SCORE_HIGH_POSITION;
                armPosition = Arm.SCORE_POSITION;
                break;
            case LAUNCH_DRONE:
                liftPosition = Lift.DRONE_LAUNCH_POSITION;
                armPosition = Arm.INTAKE_POSITION;
                break;
            case HANG_START:
                liftPosition = Lift.HANG_START_POSITION;
                armPosition = Arm.INTAKE_POSITION;
                break;
            case HANG_END:
                liftPosition = Lift.HANG_END_POSITION;
                armPosition = Arm.INTAKE_POSITION;
                break;
            default:
                armPosition = Arm.INTAKE_POSITION;
                liftPosition = Lift.INTAKE_POSITION;
        }

        moveCommand = new ParallelCommandGroup(
                new SetArmPositionCommand(arm, armPosition),
                new SetLiftPositionCommand(lift, liftPosition)
        );
        moveCommand.addRequirements(lift, arm);
    }

    @Override
    public void execute() {
        moveCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return moveCommand.isFinished();
    }

    public enum Position {
        INTAKE,
        SCORE_LOW,
        SCORE_MEDIUM,
        SCORE_HIGH,
        LAUNCH_DRONE,
        HANG_START,
        HANG_END
    }
}
