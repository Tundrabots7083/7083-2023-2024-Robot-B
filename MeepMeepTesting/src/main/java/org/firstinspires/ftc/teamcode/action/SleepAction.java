package org.firstinspires.ftc.teamcode.action;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.command.CommandEx;
import org.jetbrains.annotations.NotNull;

public class SleepAction implements Action, CommandEx {
    private final long dt;
    private long beginTs = -1;

    /**
     * Wait for the specified number of seconds.
     *
     * @param dt the number of seconds to wait.
     */
    public SleepAction(double dt) {
        this.dt = Math.round(dt * 1000.0);
    }

    /**
     * Wait for the time to expire.
     *
     * @param telemetryPacket telementry
     * @return <code>true</code> if the timer has not expired; <code>false</code> if it has expired.
     */
    @Override
    public boolean run(@NotNull TelemetryPacket telemetryPacket) {
        if (beginTs < 0) {
            beginTs = now();
        }
        final long t = now() - beginTs;
        telemetryPacket.put("[SleepAction] time remaining", dt - t);

        return t < dt;
    }

    private long now() {
        return System.currentTimeMillis();
    }

    /**
     * Returns this action.
     *
     * @return this action.
     */
    @Override
    public Action toAction() {
        return this;
    }
}

