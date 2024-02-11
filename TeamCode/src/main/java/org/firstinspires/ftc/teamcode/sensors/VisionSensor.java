package org.firstinspires.ftc.teamcode.sensors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.VisionProcessor;
import org.firstinspires.ftc.teamcode.processors.TeamElementLocation;
import org.firstinspires.ftc.vision.VisionPortal;

public class VisionSensor {

    VisionProcessor visionProcessor;

    WebcamName frontWebcamName;

    VisionPortal frontWebcamPortal;

    public VisionSensor(WebcamName frontWebcamName, Telemetry telemetry) {
        this.frontWebcamName = frontWebcamName;
        visionProcessor = new VisionProcessor(telemetry);
    }

    public void initializeVisionPortal() {
        frontWebcamPortal = VisionPortal.easyCreateWithDefaults(frontWebcamName, visionProcessor);
    }

    public boolean webcamInitialized() {
        return frontWebcamPortal != null
                && (frontWebcamPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY
                || frontWebcamPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    public TeamElementLocation getTeamElementLocation() {
        return visionProcessor.getSelection();
    }

    public void close() {
        if (frontWebcamPortal != null && frontWebcamPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            frontWebcamPortal.close();
        }
    }
}
