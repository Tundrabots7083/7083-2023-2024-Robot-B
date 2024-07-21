package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;
import org.firstinspires.ftc.teamcode.sensor.VisionSensor;

public class Camera extends Subsystem {
    private final VisionSensor visionSensor;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        this.visionSensor = new VisionSensor(hardwareMap.get(WebcamName.class, "Webcam Front"), telemetry);
    }

    /**
     * Returns an indication as to whether the camera is initialized.
     *
     * @return <code>true</code> if the camera is initialized;
     * <code>false</code> if it is not.
     */
    public boolean isInitialized() {
        return visionSensor.webcamInitialized();
    }

    /**
     * Turns the camera off.
     */
    public void close() {
        // visionSensor.close(); TODO: un-comment this once the FTC bug is fixed
    }

    /**
     * Returns the position of the team element.
     *
     * @return the location of the team element.
     */
    public TeamElementLocation getTeamElementLocation() {
        return visionSensor.getTeamElementLocation();
    }
}
