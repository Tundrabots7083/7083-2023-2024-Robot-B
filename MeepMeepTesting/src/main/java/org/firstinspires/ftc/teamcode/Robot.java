package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystem.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;

public class Robot {
    public final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();
    public final PixelCollector leftPixelCollector = new PixelCollector(PixelCollector.Location.LEFT);
    public final PixelCollector rightPixelCollector = new PixelCollector(PixelCollector.Location.RIGHT);

    private final static Robot robot = new Robot();

    public static Robot getInstance() {
        return robot;
    }
}
