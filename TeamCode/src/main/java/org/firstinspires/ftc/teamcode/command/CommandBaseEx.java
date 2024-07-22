package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Base implementation of commands. This extends the CommandBase implementation to allow the
 * creation of a RoadRunner action from the command.
 */
public abstract class CommandBaseEx extends CommandBase implements CommandEx {

    /**
     * Provides an Action facade for an FTCLib command.
     *
     * @return an Action that wraps the command.
     */
    @Override
    public Action asAction() {
        return new CommandAction(this);
    }
}
