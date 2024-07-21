package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Base implementation of commands. This extends the CommandBase implementation to allow the
 * creation of a RoadRunner action from the command.
 */
public abstract class CommandBaseEx extends CommandBase {
    /**
     * Provides an Action facade for an FTCLib command.
     *
     * @return an Action that wraps the command.
     */
    public Action asAction() {
        return new ActionWrapper(this);
    }

    /**
     * Provide a RoadRunner Action interface to an FTCLib command.
     */
    private static class ActionWrapper implements Action {
        private final Command command;
        private boolean initialized = false;

        /**
         * Instantiates the Action interface for an FTCLib command.
         * @param command
         */
        public ActionWrapper(Command command) {
            this.command = command;
        }

        /**
         * Executes the FTCLib command.
         *
         * @param telemetryPacket the telemetry to be used for sending data to the user.
         * @return <code>true</code> if the action should continue to execute;
         * <code>false</code> if the action has completed execution.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                command.initialize();
                initialized = true;
            }

            command.execute();
            boolean finished = command.isFinished();

            return !finished;
        }
    }
}
