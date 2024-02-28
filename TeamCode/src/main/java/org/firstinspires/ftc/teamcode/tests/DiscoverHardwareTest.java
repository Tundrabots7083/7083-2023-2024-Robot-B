package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.DiscoverHardware;

import java.util.Collection;

public class DiscoverHardwareTest extends Test {
    private final Collection<DiscoverHardware.DeviceInfo> devices;
    private final Telemetry telemetry;

    public DiscoverHardwareTest(DiscoverHardware hardware, Telemetry telemetry) {
        super("Hardware Discovery");

        this.telemetry = telemetry;

        this.devices = hardware.getDevices();
    }

    @Override
    public void run(Gamepad gamepad1, Gamepad gamepad2) {
        for (DiscoverHardware.DeviceInfo device : devices) {
            telemetry.addData("Device Name: ", device.deviceName);
            telemetry.addData("Device Type: ", device.deviceType);
        }

    }
}
