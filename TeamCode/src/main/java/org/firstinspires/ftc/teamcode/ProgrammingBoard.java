package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Motor;
import org.firstinspires.ftc.teamcode.tests.Test;
import org.firstinspires.ftc.teamcode.tests.TestClaw;
import org.firstinspires.ftc.teamcode.tests.TestDiscoverHardware;
import org.firstinspires.ftc.teamcode.tests.TestMotor;
import org.firstinspires.ftc.teamcode.utils.DiscoverHardware;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Programming board represents a test board that may be used to execute various
 * tests on hardware
 * without having to have a complete Robot for the testing.
 */
public class ProgrammingBoard {
    private Claw claw;
    private Motor motor;
    private DiscoverHardware hardware;

    public ProgrammingBoard() {
        // NO-OP
    }

    public void init(HardwareMap hwMap) {
        claw = new Claw("claw", "Claw");
        claw.init(hwMap);
        motor = new Motor("test_motor", "Motor");
        motor.init(hwMap);
        hardware = new DiscoverHardware(hwMap);
    }

    public List<Test> getTests() {
            final double MOTOR_SPEED = 0.5;

            ArrayList<Test> tests = new ArrayList<>();
            tests.add(new TestClaw(claw));
            tests.add(new TestMotor(motor, motor.getDescription(), MOTOR_SPEED));
            tests.add(new TestDiscoverHardware(hardware));

            return tests;
        }
}
