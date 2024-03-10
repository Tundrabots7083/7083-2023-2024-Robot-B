package org.firstinspires.ftc.teamcode.controlhub;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collection;

public class DiscoverHardware {

    private final Collection<DeviceInfo> devices;

    public DiscoverHardware(HardwareMap hwMap) {
        devices = new ArrayList<>();

        for (HardwareDevice hd : hwMap) {
            DeviceInfo di = new DeviceInfo(hd);
            devices.add(di);
        }
        /*
        devices.addAll(getDevices(hwMap.accelerationSensor));
        devices.addAll(getDevices(hwMap.analogInput));
        devices.addAll(getDevices(hwMap.colorSensor));
        devices.addAll(getDevices(hwMap.compassSensor));
        devices.addAll(getDevices(hwMap.crservo));
        devices.addAll(getDevices(hwMap.dcMotor));
        devices.addAll(getDevices(hwMap.dcMotorController));
        devices.addAll(getDevices(hwMap.gyroSensor));
        devices.addAll(getDevices(hwMap.i2cDevice));
        devices.addAll(getDevices(hwMap.i2cDeviceSynch));
        devices.addAll(getDevices(hwMap.irSeekerSensor));
        devices.addAll(getDevices(hwMap.led));
        devices.addAll(getDevices(hwMap.lightSensor));
        devices.addAll(getDevices(hwMap.opticalDistanceSensor));
        devices.addAll(getDevices(hwMap.pwmOutput));
        devices.addAll(getDevices(hwMap.servo));
        devices.addAll(getDevices(hwMap.servoController));
        devices.addAll(getDevices(hwMap.touchSensor));
        devices.addAll(getDevices(hwMap.touchSensorMultiplexer));
        devices.addAll(getDevices(hwMap.ultrasonicSensor));
        devices.addAll(getDevices(hwMap.voltageSensor));
        */

        /*
        Can look into adding other hardware devices, by getting all instances. This can work for
        IMUs and the like.
         */
    }

    /*
    private static <DEVICE_TYPE extends HardwareDevice> Collection<DeviceInfo> getDevices(HardwareMap.DeviceMapping<DEVICE_TYPE> deviceMapping) {
        Collection<DeviceInfo> devices = new ArrayList<>();

        for (Map.Entry<String, DEVICE_TYPE> entry : deviceMapping.entrySet()) {
            DeviceInfo device = new DeviceInfo(entry.getValue());
            devices.add(device);
        }

        return devices;
    }
    */

    public Collection<DeviceInfo> getDevices() {
        return devices;
    }

    public static class DeviceInfo {
        public String deviceName;
        public String deviceType;
        public HardwareDevice device;

        public DeviceInfo(HardwareDevice device) {
            this.deviceName = device.getDeviceName();
            this.deviceType = device.getClass().getName();
            this.device = device;
        }
    }
}
