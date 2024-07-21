package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.action.DisplayAction;
import org.firstinspires.ftc.teamcode.action.SleepAction;

public class Lift {
    public Action setLiftTo(Position position) {
        return new SequentialAction(
                new DisplayAction("Set Lift to " + position),
                new SleepAction(1.5)
        );
    }

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
