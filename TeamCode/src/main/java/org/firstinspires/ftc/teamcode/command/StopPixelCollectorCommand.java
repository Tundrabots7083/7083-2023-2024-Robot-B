package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;

public class StopPixelCollectorCommand extends CommandBaseEx {
    public static double SPINNER_DELAY = 250;
    private final PixelCollector pixelCollector;
    private final ElapsedTime timer = new ElapsedTime();

    public StopPixelCollectorCommand(PixelCollector pixelCollector) {
        this.pixelCollector = pixelCollector;
    }

    @Override
    public void initialize() {
        pixelCollector.setSpinnerPower(PixelCollector.SPINNER_OFF_POWER);
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.milliseconds() > SPINNER_DELAY) {
            pixelCollector.setFlapPosition(PixelCollector.FLAP_CLOSED_POSITION);
        }
    }
}
