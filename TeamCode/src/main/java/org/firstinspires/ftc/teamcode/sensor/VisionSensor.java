package org.firstinspires.ftc.teamcode.sensor;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * A vision sensor used to create and obtain data from a vision processor and determine the
 * location of the team prop.
 */
public class VisionSensor implements Sensor {

    private final VisionProcessor visionProcessor;
    private VisionPortal webcamPortal;

    /**
     * Creates a new vision sensor for the webcam with the given name.
     *
     * @param webcamName the name of the webcam.
     * @param telemetry  the telemetry used to provide output on the driver station.
     */
    public VisionSensor(WebcamName webcamName, Telemetry telemetry) {
        visionProcessor = new VisionProcessor(telemetry);
        webcamPortal = VisionPortal.easyCreateWithDefaults(webcamName, visionProcessor);
        FtcDashboard.getInstance().startCameraStream(webcamPortal, 0);
        CameraStreamServer.getInstance().setSource(webcamPortal);
    }

    @Override
    public void update() {
        // NO-OP
    }

    /**
     * Returns indication as to whether the webcam portal used by the vision sensor is or is not
     * initialized.
     *
     * @return <code>true</code> if the vision sensor is initialized;
     * <code>false</code> if it is not.
     */
    public boolean webcamInitialized() {
        return webcamPortal != null && webcamPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Returns the location of the team element.
     *
     * @return the location of the team element.
     */
    public TeamElementLocation getTeamElementLocation() {
        return visionProcessor.getSelection();
    }

    /**
     * Closes the webcam portal used by the vision sensor. The vision sensor should not be used
     * once the webcam portal has been closed.
     */
    public void close() {
        if (webcamPortal != null && webcamPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            FtcDashboard.getInstance().stopCameraStream();
            webcamPortal.close();
            webcamPortal = null;
        }
    }
}
