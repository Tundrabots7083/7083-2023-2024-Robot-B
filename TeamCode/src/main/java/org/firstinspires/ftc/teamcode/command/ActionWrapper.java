package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;

/**
 * Provide a RoadRunner Action interface to an FTCLib command.
 */
public class ActionWrapper implements Action {
    private final Command command;
    private boolean initialized = false;

    /**
     * Instantiates the Action interface for an FTCLib command.
     *
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
        if (finished) {
            command.end(false);
        }

        return !finished;
    }
}
