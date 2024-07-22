package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * WebCam attached to the robot
 */
public class WebCam extends Subsystem {
    private final VisionProcessor visionProcessor;
    private VisionPortal webcamPortal;

    /**
     * Creates a new WebCam.
     *
     * @param deviceName  name of the WebCam device
     * @param hardwareMap mapping of the hardware on the robot
     * @param telemetry   telemetry to be used for displaying output data
     */
    public WebCam(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, deviceName);
        visionProcessor = new VisionProcessor(telemetry);
        webcamPortal = VisionPortal.easyCreateWithDefaults(webcamName, visionProcessor);
        FtcDashboard.getInstance().startCameraStream(webcamPortal, 0);
        CameraStreamServer.getInstance().setSource(webcamPortal);
    }

    /**
     * Returns an indication as to whether the camera is initialized.
     *
     * @return <code>true</code> if the camera is initialized;
     * <code>false</code> if it is not.
     */
    public boolean isInitialized() {
        return webcamPortal != null && webcamPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Turns the camera off.
     */
    public void close() {
        if (webcamPortal != null && webcamPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            FtcDashboard.getInstance().stopCameraStream();
            webcamPortal.close();
            webcamPortal = null;
        }
    }

    /**
     * Returns the position of the team element.
     *
     * @return the location of the team element.
     */
    public TeamElementLocation getTeamElementLocation() {
        return visionProcessor.getSelection();
    }
}
