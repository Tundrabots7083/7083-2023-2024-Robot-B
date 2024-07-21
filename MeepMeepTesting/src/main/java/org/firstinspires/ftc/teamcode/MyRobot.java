package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;
import org.firstinspires.ftc.teamcode.subsystem.ScoringSubsystem;

public class MyRobot {
    public final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();
    public final PixelCollector leftPixelCollector = new PixelCollector(PixelCollector.Location.LEFT);
    public final PixelCollector rightPixelCollector = new PixelCollector(PixelCollector.Location.RIGHT);

    private final static MyRobot robot = new MyRobot();

    public static MyRobot getInstance() {
        return robot;
    }
}
