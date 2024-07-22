package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;
import org.firstinspires.ftc.teamcode.subsystem.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
public class Robot {
    private static Robot robot = null;

    public final Telemetry telemetry;

    // Mechanisms
    public final MecanumDrive mecanumDrive;
    public final DroneLauncher droneLauncher;
    public final Arm arm;
    public final Lift lift;
    public final ScoringSubsystem scoringSubsystem;
    public final PixelCollector leftPixelCollector, rightPixelCollector;
    public final Webcam webcam;

    /**
     * Creates a new instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     */
    private Robot(HardwareMap hardwareMap, Telemetry telemetry, OpModeType opModeType) {
        robot = this;
        this.telemetry = telemetry;

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        droneLauncher = new DroneLauncher(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        leftPixelCollector = new PixelCollector("collectorLeft", "Left pixel collector", hardwareMap, telemetry, true);
        rightPixelCollector = new PixelCollector("collectorRight", "Right pixel collector", hardwareMap, telemetry, false);

        // Instantiate the scoring subsystem that manages the lift and arm
        scoringSubsystem = new ScoringSubsystem(lift, arm, telemetry);

        if (opModeType == OpModeType.AUTO) {
            // Create the vision sensor
            webcam = new Webcam("Webcam Front", hardwareMap, telemetry);
        } else {
            webcam = null;
        }

        this.telemetry.addLine("[Robot] initialized");
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     *
     * @return the robot instance
     */
    public static Robot init(HardwareMap hardwareMap, Telemetry telemetry) {
        return init(hardwareMap, telemetry, OpModeType.TELEOP);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     * @return the robot instance
     */
    public static Robot init(HardwareMap hardwareMap, Telemetry telemetry, OpModeType opModeType) {
        robot = new Robot(hardwareMap, telemetry, opModeType);
        return robot;
    }

    /**
     * Gets the singleton instance of the robot.
     */
    public static Robot getInstance() {
        return robot;
    }

    // enum to specify opmode type
    public enum OpModeType {
        TELEOP,
        AUTO
    }
}
