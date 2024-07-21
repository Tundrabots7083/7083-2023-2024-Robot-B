package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.PixelCollector;

public class MyRobot {
    public final Lift lift = new Lift();
    public final PixelCollector leftPixelCollector = new PixelCollector(PixelCollector.Location.LEFT);
    public final PixelCollector rightPixelCollector = new PixelCollector(PixelCollector.Location.RIGHT);

    private final static MyRobot robot = new MyRobot();

    public static MyRobot getInstance() {
        return robot;
    }
}
