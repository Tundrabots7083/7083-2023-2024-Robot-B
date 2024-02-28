package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.DiscoverHardware;

import java.util.ArrayList;
import java.util.List;

/**
 * Programming board represents a test board that may be used to execute various
 * tests on hardware
 * without having to have a complete Robot for the testing.
 */
public class ProgrammingBoard {
    private DcMotorEx motor;
    private DiscoverHardware hardware;
    private Telemetry telemetry;

    public ProgrammingBoard() {
        // NO-OP
    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = hwMap.get(DcMotorEx.class, "test_motor");
        hardware = new DiscoverHardware(hwMap);
    }

    public List<Test> getTests() {
        final double MOTOR_SPEED = 0.5;

        ArrayList<Test> tests = new ArrayList<>();
        tests.add(new MotorTest(motor, "motor", MOTOR_SPEED));
        tests.add(new DiscoverHardwareTest(hardware, telemetry));

        return tests;
    }
}
