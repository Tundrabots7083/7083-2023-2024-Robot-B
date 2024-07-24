package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * DroneLauncher manages the launching of the drone (paper airplane)
 */
@Config
public class DroneLauncher extends SubsystemBaseEx {

    public static double SERVO_LAUNCH_START_ANGLE = 0.0;
    public static double SERVO_LAUNCH_ANGLE_HIGH = 0.25500334; // 0.28; // 52.5º // 0.27166667; // 49º
    public static double SERVO_LAUNCH_ANGLE_LOW = 0.25;   // 46.5º
    public static boolean SERVO_LAUNCH_ANGLE_USE_HIGH_ANGLE = true;
    public static double SERVO_RELEASE_POS = 0.1;

    public static double MIN_RELEASE_ANGLE = 0.0;
    public static double MAX_RELEASE_ANGLE = 0.5;
    public static double MIN_LAUNCH_ANGLE = 0.0;
    public static double MAX_LAUNCH_ANGLE = 0.5;

    private final Telemetry telemetry;
    private final ServoEx releaseServo;
    private final ServoEx angleServo;

    /**
     * Creates the drone launcher
     *
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        releaseServo = new SimpleServo(hardwareMap, "droneLauncher", MIN_RELEASE_ANGLE, MAX_RELEASE_ANGLE);
        releaseServo.setInverted(false);

        angleServo = new SimpleServo(hardwareMap, "dronePosition", MIN_LAUNCH_ANGLE, MAX_LAUNCH_ANGLE);
        angleServo.setInverted(true);
        angleServo.setPosition(SERVO_LAUNCH_START_ANGLE);
    }

    /**
     * Moves the drone launcher into the down position
     */
    public void setToStartAngle() {
        angleServo.setPosition(SERVO_LAUNCH_START_ANGLE);
    }

    /**
     * Moves the drone launcher into launch position
     */
    public void setToLaunchAngle() {
        double servoAnglePosition = SERVO_LAUNCH_ANGLE_USE_HIGH_ANGLE ? SERVO_LAUNCH_ANGLE_HIGH : SERVO_LAUNCH_ANGLE_LOW;
        angleServo.setPosition(servoAnglePosition);
    }

    /**
     * Launches the drone, hopefully scoring a lot of points in the process.
     */
    public void launchDrone() {
        releaseServo.setPosition(SERVO_RELEASE_POS);
    }
}