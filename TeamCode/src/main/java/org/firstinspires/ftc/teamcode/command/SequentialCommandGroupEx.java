package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class SequentialCommandGroupEx extends SequentialCommandGroup implements CommandEx {

    /**
     * Provides an Action facade for an FTCLib command.
     *
     * @return an Action that wraps the command.
     */
    @Override
    public Action asAction() {
        return new ActionWrapper(this);
    }
}
