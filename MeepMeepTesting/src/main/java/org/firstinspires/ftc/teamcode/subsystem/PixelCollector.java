package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.action.DisplayAction;

public class PixelCollector {
    private final Location location;

    public PixelCollector(Location location) {
        this.location = location;
    }

    public Action depositPixel() {
        return new SequentialAction(
                new DisplayAction("Deposit " + location + " pixel")
        );
    }

    public enum Location {
        LEFT,
        RIGHT
    }
}
