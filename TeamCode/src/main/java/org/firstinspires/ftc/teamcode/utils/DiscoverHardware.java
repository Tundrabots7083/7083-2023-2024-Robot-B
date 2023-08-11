package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

public class DiscoverHardware {

    public static class DeviceInfo {
        public String deviceName;
        public String deviceType;

        public DeviceInfo(String deviceName, String deviceType) {
            this.deviceName = deviceName;
            this.deviceType = deviceType;
        }
    }

    private final Collection<DeviceInfo> devices;

    private static <DEVICE_TYPE extends HardwareDevice> Collection<DeviceInfo> getDevices(HardwareMap.DeviceMapping<DEVICE_TYPE> deviceMapping) {
        Collection<DeviceInfo> devices = new ArrayList<>();

        for (Map.Entry<String, DEVICE_TYPE> entry : deviceMapping.entrySet()) {
            DeviceInfo device = new DeviceInfo(entry.getKey(), entry.getValue().getDeviceName());
            devices.add(device);
        }

        return devices;
    }

    public DiscoverHardware(HardwareMap hardwareMap) {
        devices = new ArrayList<>();

        devices.addAll(getDevices(hardwareMap.accelerationSensor));
        devices.addAll(getDevices(hardwareMap.analogInput));
        devices.addAll(getDevices(hardwareMap.colorSensor));
        devices.addAll(getDevices(hardwareMap.compassSensor));
        devices.addAll(getDevices(hardwareMap.crservo));
        devices.addAll(getDevices(hardwareMap.dcMotor));
        devices.addAll(getDevices(hardwareMap.dcMotorController));
        devices.addAll(getDevices(hardwareMap.gyroSensor));
        devices.addAll(getDevices(hardwareMap.i2cDevice));
        devices.addAll(getDevices(hardwareMap.i2cDeviceSynch));
        devices.addAll(getDevices(hardwareMap.irSeekerSensor));
        devices.addAll(getDevices(hardwareMap.led));
        devices.addAll(getDevices(hardwareMap.lightSensor));
        devices.addAll(getDevices(hardwareMap.opticalDistanceSensor));
        devices.addAll(getDevices(hardwareMap.pwmOutput));
        devices.addAll(getDevices(hardwareMap.servo));
        devices.addAll(getDevices(hardwareMap.servoController));
        devices.addAll(getDevices(hardwareMap.touchSensor));
        devices.addAll(getDevices(hardwareMap.touchSensorMultiplexer));
        devices.addAll(getDevices(hardwareMap.ultrasonicSensor));
        devices.addAll(getDevices(hardwareMap.voltageSensor));

        /* Not found in current HardwareMap
        devices.addAll(getDevices(hardwareMap.analogOutput));
        devices.addAll(getDevices(hardwareMap.deviceInterfaceModuleSet));
        devices.addAll(getDevices(hardwareMap.legacyModuleSet));
        */
    }

    public Collection<DeviceInfo> getDevices() {
        return devices;
    }
}
