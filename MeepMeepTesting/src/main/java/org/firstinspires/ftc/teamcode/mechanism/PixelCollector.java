package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.action.DisplayAction;
import org.firstinspires.ftc.teamcode.action.SleepAction;

public class PixelCollector {
    private final Location location;

    public PixelCollector(Location location) {
        this.location = location;
    }

    public Action depositPixel() {
        return new SequentialAction(
                new DisplayAction("Deposit " + location + " pixel"),
                new SleepAction(1.5)
        );
    }

    public enum Location {
        LEFT,
        RIGHT
    }
}
