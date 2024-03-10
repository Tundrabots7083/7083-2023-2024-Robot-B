package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controlhub.DiscoverHardware;

import java.util.Collection;

public class TestDiscoverHardware extends Test {
    private final Collection<DiscoverHardware.DeviceInfo> devices;

    public TestDiscoverHardware(DiscoverHardware hardware) {
        super("Hardware Discovery");

        this.devices = hardware.getDevices();
    }

    @Override
    public void run(Gamepad gamepad1, Telemetry telemetry) {
        for (DiscoverHardware.DeviceInfo device : devices) {
            telemetry.addData("Device Name: ", device.deviceName);
            telemetry.addData("Device Type: ", device.deviceType);
        }

    }
}
