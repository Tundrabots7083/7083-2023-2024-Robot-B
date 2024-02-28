package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.DroneLauncher;

public class DroneLauncherTest extends Test {
    private final DroneLauncher droneLauncher;
    public DroneLauncherTest(DroneLauncher droneLauncher) {
        super("DroneLauncher Test");

        this.droneLauncher = droneLauncher;
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.a) {
            droneLauncher.setToAngle(DroneLauncher.SERVO_LAUNCH_START_ANGLE);
        } else if (gamepad1.b) {
            droneLauncher.setToAngle(DroneLauncher.SERVO_LAUNCH_START_ANGLE);
        } else if (gamepad1.x) {
            droneLauncher.launchDrone();
        }
    }
}
