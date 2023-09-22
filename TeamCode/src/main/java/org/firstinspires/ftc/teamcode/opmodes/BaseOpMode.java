package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controls.Navigator;

/**
 * BaseOpMode is the base class for all opmodes on the system.
 */
public abstract class BaseOpMode extends OpMode {
    protected final Robot robot = new Robot();


    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addLine("Initialization Complete");
    }
}


