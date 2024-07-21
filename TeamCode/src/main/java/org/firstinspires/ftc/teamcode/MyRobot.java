package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Camera;
import org.firstinspires.ftc.teamcode.subsystem.DroneLauncher;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.PixelCollector;
import org.firstinspires.ftc.teamcode.subsystem.ScoringSubsystem;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
public class MyRobot {
    private static MyRobot robot = null;

    public final Telemetry telemetry;

    // Mechanisms
    public final MecanumDrive mecanumDrive;
    public final DroneLauncher droneLauncher;
    public final ScoringSubsystem scoringSubsystem;
    public final PixelCollector leftPixelCollector, rightPixelCollector;
    public final Camera webCam;

    /**
     * Creates a new instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  Autonomous or TeleOp OpMode
     */
    private MyRobot(HardwareMap hardwareMap, Telemetry telemetry, OpModeType opModeType) {
        robot = this;
        this.telemetry = telemetry;

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        Lift lift = new Lift(hardwareMap, telemetry);
        Arm arm = new Arm(hardwareMap, telemetry);
        scoringSubsystem = new ScoringSubsystem(lift, arm, telemetry);
        droneLauncher = new DroneLauncher(hardwareMap, telemetry);

        leftPixelCollector = new PixelCollector(PixelCollector.Location.LEFT, hardwareMap, telemetry);
        rightPixelCollector = new PixelCollector(PixelCollector.Location.RIGHT, hardwareMap, telemetry);

        webCam = opModeType == OpModeType.AUTO ? new Camera(hardwareMap, telemetry) : null;

        this.telemetry.addLine("[Robot] initialized");
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @return the robot instance
     */
    public static MyRobot init(HardwareMap hardwareMap, Telemetry telemetry) {
        return init(hardwareMap, telemetry, OpModeType.TELEOP);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opmodeType  Autonomous or TeleOp OpMode.
     * @return the robot instance
     */
    public static MyRobot init(HardwareMap hardwareMap, Telemetry telemetry, OpModeType opmodeType) {
        robot = new MyRobot(hardwareMap, telemetry, opmodeType);
        return robot;
    }


    /**
     * Gets the singleton instance of the robot.
     *
     * @return the robot instance.
     */
    public static MyRobot getInstance() {
        return robot;
    }

    /**
     * Type of OpMode the robot is being instantiated for.
     */
    public enum OpModeType {
        AUTO,
        TELEOP
    }
}
