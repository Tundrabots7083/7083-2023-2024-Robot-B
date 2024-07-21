package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;

public class IntakePixelCommand extends CommandBase {
    public static double FLAP_DELAY = 250;
    private final PixelCollector pixelCollector;
    private final ElapsedTime timer = new ElapsedTime();

    public IntakePixelCommand(PixelCollector pixelCollector) {
        this.pixelCollector = pixelCollector;
        addRequirements(pixelCollector);
    }

    @Override
    public void initialize() {
        timer.reset();
        pixelCollector.setFlapPosition(PixelCollector.FLAP_OPENED_POSITION);
    }

    @Override
    public void execute() {
        if (timer.milliseconds() > FLAP_DELAY) {
            pixelCollector.setSpinnerPower(PixelCollector.SPINNER_COLLECTING_POWER);
        }
    }
}
