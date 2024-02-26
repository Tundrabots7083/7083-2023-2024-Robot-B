package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;
import org.firstinspires.ftc.teamcode.processors.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class VisionSensor implements Sensor {

    VisionProcessor visionProcessor;

    WebcamName webcamName;

    VisionPortal webcamPortal;

    public VisionSensor(WebcamName webcamName, Telemetry telemetry) {
        this.webcamName = webcamName;
        visionProcessor = new VisionProcessor(telemetry);
    }

    @Override
    public void update() {
        // NO-OP
    }

    public void initializeVisionPortal() {
        webcamPortal = VisionPortal.easyCreateWithDefaults(webcamName, visionProcessor);
    }

    public boolean webcamInitialized() {
        return webcamPortal != null
                && (webcamPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY
                || webcamPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    public TeamElementLocation getTeamElementLocation() {
        return visionProcessor.getSelection();
    }

    public void close() {
        if (webcamPortal != null && webcamPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            webcamPortal.close();
        }
    }
}
