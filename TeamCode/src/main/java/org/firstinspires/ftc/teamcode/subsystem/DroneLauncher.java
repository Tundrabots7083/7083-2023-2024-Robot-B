package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DroneLauncher extends Subsystem {
    public static double SERVO_LAUNCH_START_ANGLE = 0.0;
    public static double SERVO_LAUNCH_ANGLE_HIGH = 0.25500334; // 0.28; // 52.5º // 0.27166667; // 49º
    public static double SERVO_LAUNCH_ANGLE_LOW = 0.25;   // 46.5º
    public static boolean SERVO_LAUNCH_ANGLE_USE_HIGH_ANGLE = true;
    public static double SERVO_RELEASE_POS_INIT = 0.0;
    public static double SERVO_RELEASE_POS = 0.1;

    private final ServoEx angleServo;
    private final ServoEx releaseServo;

    public DroneLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        super(telemetry);

        releaseServo = new SimpleServo(hardwareMap, "droneLauncher", SERVO_RELEASE_POS_INIT, SERVO_RELEASE_POS);
        angleServo = new SimpleServo(hardwareMap, "dronePosition", SERVO_LAUNCH_START_ANGLE, SERVO_LAUNCH_ANGLE_HIGH);
        angleServo.setInverted(true);
        angleServo.setPosition(SERVO_LAUNCH_START_ANGLE);

        telemetry.addLine("[DRONE LAUNCHER] initialized");
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
